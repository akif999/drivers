// Package mcp2515 implements a driver for the MCP2515 CAN Controller.
//
// Datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/MCP2515-Stand-Alone-CAN-Controller-with-SPI-20001801J.pdf
//
// Reference: https://github.com/coryjfowler/MCP_CAN_lib
package mcp2515 // import "tinygo.org/x/drivers/mcp2515"

import (
	"errors"
	"fmt"
	"machine"
	"time"

	"tinygo.org/x/drivers"
)

// Device wraps MCP2515 SPI CAN Module.
type Device struct {
	bus     drivers.SPI
	cs      machine.Pin
	mcpMode byte
}

// New returns a new MCP2515 driver. Pass in a fully configured SPI bus.
func New(b drivers.SPI, csPin machine.Pin) *Device {
	d := &Device{
		bus: b,
		cs:  csPin,
	}

	return d
}

// Configure sets up the device for communication
func (d *Device) Configure() {
	d.cs.Configure(machine.PinConfig{Mode: machine.PinOutput})
}

const beginTimeoutValue int = 10

// Begin starts the CAN controller.
func (d *Device) Begin(speed byte, clock byte) error {
	timeOutCount := 0
	for {
		err := d.init(speed, clock)
		if err == nil {
			break
		}
		timeOutCount++
		if timeOutCount >= beginTimeoutValue {
			return err
		}
	}
	return nil
}

// Received returns true if CAN message is received
func (d *Device) Received() bool {
	res, err := d.readStatus()
	if err != nil {
		panic(err)
	}
	// if RX STATUS INSTRUCTION  result is not 0x00 (= No RX message)
	// TODO: reconsider this logic
	return (res & mcpStatRxifMask) != 0x00
}

type CANMsg struct {
	ID   uint32
	Dlc  uint8
	Data []byte
}

// Rx returns received CAN message
func (d *Device) Rx() (*CANMsg, error) {
	msg := &CANMsg{}
	err := d.readMsg(msg)
	return msg, err
}

// Tx transmits CAN Message
func (d *Device) Tx(canid uint32, dlc uint8, data []byte) error {
	// TODO: add ext, rtrBit, waitSent
	timeoutCount := 0

	var bufNum, res uint8
	var err error
	res = mcpAlltxbusy
	for res == mcpAlltxbusy && (timeoutCount < timeoutvalue) {
		if timeoutCount > 0 {
			time.Sleep(time.Microsecond * 10)
		}
		bufNum, res, err = d.getNextFreeTxBuf()
		if err != nil {
			return err
		}
		timeoutCount++
	}
	if timeoutCount == timeoutvalue {
		return fmt.Errorf("Tx: Tx timeout")
	}
	err = d.writeCANMsg(bufNum, canid, 0, 0, dlc, data)
	if err != nil {
		return err
	}

	return nil
}

func (d *Device) init(speed, clock byte) error {
	err := d.Reset()
	if err != nil {
		return err
	}

	if err := d.setCANCTRLMode(modeConfig); err != nil {
		return fmt.Errorf("setCANCTRLMode %s: ", err)
	}
	time.Sleep(time.Millisecond * 10)

	// set baudrate
	if err := d.configRate(speed, clock); err != nil {
		return fmt.Errorf("configRate %s: ", err)
	}
	time.Sleep(time.Millisecond * 10)

	if err := d.initCANBuffers(); err != nil {
		return fmt.Errorf("initCANBuffers: %s ", err)
	}
	if err := d.setRegister(mcpCANINTE, mcpRX0IF|mcpRX1IF); err != nil {
		return fmt.Errorf("setRegister: %s ", err)
	}
	if err := d.modifyRegister(mcpRXB0CTRL, mcpRxbRxMask|mcpRxbBuktMask, mcpRxbRxStdExt|mcpRxbBuktMask); err != nil {
		return fmt.Errorf("modifyRegister: %s ", err)
	}
	if err := d.modifyRegister(mcpRXB1CTRL, mcpRxbRxMask, mcpRxbRxStdExt); err != nil {
		return fmt.Errorf("modifyRegister: %s ", err)
	}

	if err := d.setMode(modeNormal); err != nil {
		return fmt.Errorf("setMode %s: ", err)
	}
	time.Sleep(time.Millisecond * 10)

	return nil
}

// Reset resets mcp2515
func (d *Device) Reset() error {
	d.cs.Low()
	defer d.cs.High()
	_, err := d.spiReadWrite([]byte{mcpReset})
	// time.Sleep(time.Microsecond * 4)
	d.cs.High()
	if err != nil {
		return err
	}

	time.Sleep(time.Millisecond * 10)

	return nil
}

func (d *Device) setCANCTRLMode(newMode byte) error {
	// If the chip is asleep and we want to change mode then a manual wake needs to be done
	// This is done by setting the wake up interrupt flag
	// This undocumented trick was found at https://github.com/mkleemann/can/blob/master/can_sleep_mcp2515.c
	m, err := d.getMode()
	if err != nil {
		return err
	}
	if m == modeSleep && newMode != modeSleep {
		r, err := d.readRegister(mcpCANINTE)
		if err != nil {
			return err
		}
		wakeIntEnabled := (r & mcpWAKIF) == 0x00
		if !wakeIntEnabled {
			d.modifyRegister(mcpCANINTE, mcpWAKIF, mcpWAKIF)
		}
		// Set wake flag (this does the actual waking up)
		d.modifyRegister(mcpCANINTF, mcpWAKIF, mcpWAKIF)

		// Wait for the chip to exit SLEEP and enter LISTENONLY mode.

		// If the chip is not connected to a CAN bus (or the bus has no other powered nodes) it will sometimes trigger the wake interrupt as soon
		// as it's put to sleep, but it will stay in SLEEP mode instead of automatically switching to LISTENONLY mode.
		// In this situation the mode needs to be manually set to LISTENONLY.

		if err := d.requestNewMode(modeListenOnly); err != nil {
			return err
		}

		// Turn wake interrupt back off if it was originally off
		if !wakeIntEnabled {
			d.modifyRegister(mcpCANINTE, mcpWAKIF, 0)
		}
	}

	// Clear wake flag
	d.modifyRegister(mcpCANINTF, mcpWAKIF, 0)

	return d.requestNewMode(newMode)
}

func (d *Device) setMode(opMode byte) error {
	if opMode != modeSleep {
		d.mcpMode = opMode
	}

	err := d.setCANCTRLMode(opMode)
	if err != nil {
		return err
	}

	return nil
}

func (d *Device) getMode() (byte, error) {
	r, err := d.readRegister(mcpCANSTAT)
	if err != nil {
		return 0, err
	}
	return r & modeMask, nil
}

func (d *Device) configRate(speed, clock byte) error {
	// TODO: add another baudrate
	var cfg1, cfg2, cfg3 byte
	set := true
	switch clock {
	case Clock16MHz:
		switch speed {
		case CAN500kBps:
			cfg1 = mcp16mHz500kBpsCfg1
			cfg2 = mcp16mHz500kBpsCfg2
			cfg3 = mcp16mHz500kBpsCfg3
		case CAN1000kBps:
			cfg1 = mcp16mHz1000kBpsCfg1
			cfg2 = mcp16mHz1000kBpsCfg2
			cfg3 = mcp16mHz1000kBpsCfg3
		default:
			set = false
		}
	case Clock8MHz:
		switch speed {
		case CAN500kBps:
			cfg1 = mcp8mHz500kBpsCfg1
			cfg2 = mcp8mHz500kBpsCfg2
			cfg3 = mcp8mHz500kBpsCfg3
		case CAN1000kBps:
			cfg1 = mcp8mHz1000kBpsCfg1
			cfg2 = mcp8mHz1000kBpsCfg2
			cfg3 = mcp8mHz1000kBpsCfg3
		default:
			set = false
		}
	default:
		set = false
	}
	if !set {
		return errors.New("invalid parameter")
	}
	if err := d.setRegister(mcpCNF1, cfg1); err != nil {
		return err
	}
	if err := d.setRegister(mcpCNF2, cfg2); err != nil {
		return err
	}
	if err := d.setRegister(mcpCNF3, cfg3); err != nil {
		return err
	}

	return nil
}

func (d *Device) initCANBuffers() error {
	a1 := byte(mcpTXB0CTRL)
	a2 := byte(mcpTXB1CTRL)
	a3 := byte(mcpTXB2CTRL)
	for i := 0; i < 14; i++ {
		if err := d.setRegister(a1, 0); err != nil {
			return err
		}
		if err := d.setRegister(a2, 0); err != nil {
			return err
		}
		if err := d.setRegister(a3, 0); err != nil {
			return err
		}
		a1++
		a2++
		a3++
	}

	if err := d.setRegister(mcpRXB0CTRL, 0); err != nil {
		return err
	}
	if err := d.setRegister(mcpRXB1CTRL, 0); err != nil {
		return err
	}

	return nil
}

func (d *Device) readMsg(msg *CANMsg) error {
	status, err := d.readRxTxStatus()
	if err != nil {
		return err
	}
	if (status & mcpRX0IF) == 0x01 {
		msg.ID, _, _, msg.Dlc, msg.Data, err = d.readRxBuffer(mcpReadRx0)
		if err != nil {
			return err
		}
	} else if (status & mcpRX1IF) == 0x02 {
		msg.ID, _, _, msg.Dlc, msg.Data, err = d.readRxBuffer(mcpReadRx1)
		if err != nil {
			return err
		}
	} else {
		return fmt.Errorf("readMsg: nothing is received")
	}

	return nil
}

func (d *Device) readRxBuffer(loadAddr uint8) (uint32, uint8, uint8, uint8, []byte, error) {
	d.cs.Low()
	defer d.cs.High()
	_, err := d.spiReadWrite([]byte{loadAddr})
	if err != nil {
		return 0, 0, 0, 0, []byte{}, err
	}
	var buf [4]byte
	for i := 0; i < 4; i++ {
		b, err := d.spiRead()
		if err != nil {
			return 0, 0, 0, 0, []byte{}, err
		}
		buf[i] = b[0]
	}
	id := uint32((uint32(buf[0]) << 3) + (uint32(buf[1]) >> 5))
	ext := uint8(0)
	if (buf[1] & mcpTxbExideM) == mcpTxbExideM {
		// extended id
		id = uint32(uint32(id<<2) + uint32(buf[1]&0x03))
		id = uint32(uint32(id<<8) + uint32(buf[2]))
		id = uint32(uint32(id<<8) + uint32(buf[3]))
		ext = 1
	}
	b, err := d.spiRead()
	if err != nil {
		return 0, 0, 0, 0, []byte{}, err
	}
	msgSize := b[0]
	dlc := uint8(msgSize & mcpDlcMask)
	rtrBit := uint8(0)
	if (msgSize & mcpRtrMask) == 0x40 {
		rtrBit = 1
	}
	data := []byte{}
	for i := 0; i < int(dlc) && i < canMaxCharInMessage; i++ {
		b, err := d.spiRead()
		if err != nil {
			return 0, 0, 0, 0, []byte{}, err
		}
		data = append(data, b[0])

	}
	d.cs.High()

	return id, ext, rtrBit, dlc, data, nil
}

func (d *Device) getNextFreeTxBuf() (uint8, uint8, error) {
	status, err := d.readStatus()
	if err != nil {
		return 0, mcpAlltxbusy, err
	}
	status &= mcpStatTxPendingMask

	bufNum := uint8(0x00)

	if status == mcpStatTxPendingMask {
		return 0, mcpAlltxbusy, nil
	}

	for i := 0; i < int(mcpNTxbuffers-nReservedTx(0)); i++ {
		if (status & txStatusPendingFlag(uint8(i))) == 0 {
			bufNum = txCtrlReg(uint8(i)) + 1
			d.modifyRegister(mcpCANINTF, txIfFlag(uint8(i)), 0)
			return bufNum, mcp2515Ok, nil
		}
	}

	return 0, mcpAlltxbusy, nil
}

func (d *Device) writeCANMsg(bufNum uint8, canid uint32, ext, rtrBit, dlc uint8, data []byte) error {
	loadAddr := txSidhToLoad(bufNum)

	if rtrBit == 1 {
		dlc |= mcpRtrMask
	} else {
		dlc |= (0)
	}
	txBufData := IDtoBuf(ext, canid)
	txBufData = append(txBufData, dlc)
	txBufData = append(txBufData, data...)

	d.cs.Low()
	defer d.cs.High()
	_, err := d.spiReadWrite([]byte{loadAddr})
	if err != nil {
		return err
	}
	for _, data := range txBufData {
		err := d.spiWrite([]byte{data})
		if err != nil {
			return err
		}
	}
	d.cs.High()

	err = d.startTransmission(bufNum)
	if err != nil {
		return err
	}

	return nil
}

func (d *Device) startTransmission(bufNum uint8) error {
	d.cs.Low()
	defer d.cs.High()
	_, err := d.spiReadWrite([]byte{txSidhToRTS(bufNum)})
	if err != nil {
		return err
	}
	d.cs.High()

	return nil
}

func IDtoBuf(ext uint8, id uint32) []byte {
	buf := []byte{}

	id = id & 0x0FFFF

	if ext == 1 {
		// TODO: add Extended ID
		buf = append(buf, 0)
		buf = append(buf, 0)
		buf = append(buf, 0)
		buf = append(buf, 0)
	} else {
		buf = append(buf, byte(id>>3))
		buf = append(buf, byte((id&0x07)<<5))
		buf = append(buf, 0)
		buf = append(buf, 0)
	}

	return buf
}

func nReservedTx(number uint8) uint8 {
	if number < mcpNTxbuffers {
		return number
	}
	return mcpNTxbuffers - 1
}

func txStatusPendingFlag(i uint8) uint8 {
	ret := uint8(0)
	switch i {
	case 0:
		ret = mcpStatTx0Pending
	case 1:
		ret = mcpStatTx1Pending
	case 2:
		ret = mcpStatTx2Pending
	}
	return ret
}

func txCtrlReg(status uint8) uint8 {
	ret := uint8(0)
	switch status {
	case 0:
		ret = mcpTXB0CTRL
	case 1:
		ret = mcpTXB1CTRL
	case 2:
		ret = mcpTXB2CTRL
	}
	return ret
}

func txIfFlag(i uint8) uint8 {
	ret := uint8(0)
	switch i {
	case 0:
		ret = mcpTX0IF
	case 1:
		ret = mcpTX1IF
	case 2:
		ret = mcpTX2IF
	}
	return ret
}

func txSidhToSidh(i uint8) uint8 {
	ret := uint8(0)
	switch i {
	case mcpTX0IF:
		ret = mcpTXB0SIDH
	case mcpTX1IF:
		ret = mcpTXB1SIDH
	case mcpTX2IF:
		ret = mcpTXB2SIDH
	}
	return ret
}

func txSidhToRTS(i uint8) uint8 {
	ret := uint8(0)
	switch i {
	case mcpTXB0SIDH:
		ret = mcpRtsTx0
	case mcpTXB1SIDH:
		ret = mcpRtsTx1
	case mcpTXB2SIDH:
		ret = mcpRtsTx2
	}
	return ret
}

func txSidhToLoad(i uint8) uint8 {
	ret := uint8(0)
	switch i {
	case mcpTXB0SIDH:
		ret = mcpLoadTx0
	case mcpTXB1SIDH:
		ret = mcpLoadTx1
	case mcpTXB2SIDH:
		ret = mcpLoadTx2
	}
	return ret
}

func (d *Device) setRegister(addr, value byte) error {
	d.cs.Low()
	defer d.cs.High()
	_, err := d.spiReadWrite([]byte{mcpWrite})
	if err != nil {
		return err
	}
	_, err = d.spiReadWrite([]byte{addr})
	if err != nil {
		return err
	}
	_, err = d.spiReadWrite([]byte{value})
	if err != nil {
		return err
	}
	// time.Sleep(time.Microsecond * 4)
	d.cs.High()

	return nil
}

func (d *Device) readRegister(addr byte) (byte, error) {
	d.cs.Low()
	defer d.cs.High()
	_, err := d.spiReadWrite([]byte{mcpRead})
	if err != nil {
		return 0, err
	}
	_, err = d.spiReadWrite([]byte{addr})
	if err != nil {
		return 0, err
	}
	r, err := d.spiRead()
	if err != nil {
		return 0, err
	}
	if len(r) < 1 {
		return 0, errors.New("read SPI buffer is too short")
	}
	// time.Sleep(time.Microsecond * 4)
	d.cs.High()
	return r[0], nil
}

func (d *Device) modifyRegister(addr, mask, data byte) error {
	d.cs.Low()
	defer d.cs.High()
	_, err := d.spiReadWrite([]byte{mcpBitMod})
	if err != nil {
		return err
	}
	_, err = d.spiReadWrite([]byte{addr})
	if err != nil {
		return err
	}
	_, err = d.spiReadWrite([]byte{mask})
	if err != nil {
		return err
	}
	_, err = d.spiReadWrite([]byte{data})
	if err != nil {
		return err
	}
	// time.Sleep(time.Microsecond * 4)
	d.cs.High()

	return nil
}

func (d *Device) requestNewMode(newMode byte) error {
	s := time.Now()
	for {
		err := d.modifyRegister(mcpCANCTRL, modeMask, newMode)
		if err != nil {
			return err
		}
		r, err := d.readRegister(mcpCANSTAT)
		if err != nil {
			return err
		}
		if r&modeMask == newMode {
			return nil
		} else if e := time.Now(); e.Sub(s) > 200*time.Millisecond {
			return errors.New("requestNewMode max time expired")
		}
	}
}

func (d *Device) readStatus() (byte, error) {
	d.cs.Low()
	defer d.cs.High()
	_, err := d.spiReadWrite([]byte{mcpReadStatus})
	if err != nil {
		return 0, err
	}
	ret, err := d.spiRead()
	if err != nil {
		return 0, err
	}
	d.cs.High()

	return ret[0], nil
}

func (d *Device) readRxTxStatus() (byte, error) {
	status, err := d.readStatus()
	if err != nil {
		return 0, err
	}
	ret := status & (mcpStatTxifMask | mcpStatRxifMask)
	if (status & mcpStatTx0if) == 0x08 {
		ret |= mcpTX0IF
	}
	if (status & mcpStatTx1if) == 0x20 {
		ret |= mcpTX1IF
	}
	if (status & mcpStatTx2if) == 0x80 {
		ret |= mcpTX2IF
	}
	ret |= ret & mcpStatRxifMask

	return ret, nil
}

func (d *Device) spiReadWrite(w []byte) ([]byte, error) {
	r := make([]byte, len(w))
	err := d.bus.Tx(w, r)
	if err != nil {
		return nil, fmt.Errorf("spiReadWrite: %s", err)
	}
	return r, nil
}

func (d *Device) spiRead() ([]byte, error) {
	r := make([]byte, 1)
	err := d.bus.Tx(nil, r)
	if err != nil {
		return nil, fmt.Errorf("spiRead: %s", err)
	}
	return r, nil
}

func (d *Device) spiWrite(w []byte) error {
	return d.bus.Tx(w, nil)
}

func (d *Device) dumpMode() error {
	m, err := d.getMode()
	if err != nil {
		return err
	}
	fmt.Printf("Mode: %02X\r\n", m)

	return nil
}

func (d *Device) dumpRegister(addr byte) error {
	r, err := d.readRegister(addr)
	if err != nil {
		return err
	}
	fmt.Printf("Register: %02X = %02X\r\n", addr, r)

	return nil
}
