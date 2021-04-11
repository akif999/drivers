package main

import (
	"fmt"
	"log"
	"machine"
	"time"

	"tinygo.org/x/drivers/mcp2515"
)

func main() {
	can := mcp2515.New(machine.SPI0, machine.D5)
	can.Configure()
	err := can.Begin(mcp2515.CAN500kBps, mcp2515.Clock8MHz)
	if err != nil {
		panic(err)
	}

	led := machine.LED
	led.Configure(machine.PinConfig{Mode: machine.PinOutput})

	for {
		led.Toggle()
		err := can.Tx(0x111, 8, []byte{0x00, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA})
		if err != nil {
			log.Fatal(err)
		}
		if can.Received() {
			msg, err := can.Rx()
			if err != nil {
				panic(err)
			}
			fmt.Printf("CAN-ID: %03X dlc: %d data: ", msg.ID, msg.Dlc)
			for _, b := range msg.Data {
				fmt.Printf("%02X ", b)
			}
			fmt.Println()
		}
		time.Sleep(time.Millisecond * 500)
	}
}
