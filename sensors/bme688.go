package sensors

import (
	"log"
	"time"

	"github.com/atgrier/stratux/sensors/bme688"

	"github.com/kidoman/embd"
)

type BME688 struct {
	sensor      *bme688.BME688
	temperature float64
	pressure    float64
	humidity    float64
	running     bool
}

func NewBME688(i2cbus *embd.I2CBus) (*BME688, error) {

	bme := bme688.BME688{Address: bme688.Address, Config: bme688.Config{
		Temperature: bme688.Sampling8X,
		Pressure:    bme688.Sampling2X,
		Humidity:    bme688.Sampling2X,
		IIR:         bme688.Coeff0,
	}, Bus: i2cbus} //new sensor
	// retry to connect until sensor connected
	var connected bool
	for n := 0; n < 5; n++ {
		if bme.Connected() {
			connected = true
		} else {
			time.Sleep(time.Millisecond)
		}
	}
	if !connected {
		return nil, bme688.ErrNotConnected
	}
	err := bme.Configure(bme.Config)
	if err != nil {
		return nil, err
	}
	newbme := BME688{sensor: &bme}

	go newbme.run()
	return &newbme, nil
}
func (bme *BME688) run() {
	bme.running = true
	clock := time.NewTicker(100 * time.Millisecond)
	for bme.running {
		for _ = range clock.C {
			var p, _ = bme.sensor.ReadPressure()
			// log.Printf("Pressure: %f\n", p)
			bme.pressure = p
			var t, _ = bme.sensor.ReadTemperature()
			// log.Printf("Temperature: %f\n", t)
			bme.temperature = t
			var h, _ = bme.sensor.ReadHumidity()
			// log.Printf("Humidity: %f\n", h)
			bme.humidity = h
		}

	}
}

func (bme *BME688) Close() {
	bme.running = false
	bme.sensor.Config.Mode = bme688.Sleep
	_ = bme.sensor.Configure(bme.sensor.Config)
}

// Temperature returns the current temperature in degrees C measured by the BME280
func (bme *BME688) Temperature() (float64, error) {
	if !bme.running {
		return 0, bme688.ErrNotConnected
	}

	return bme.temperature, nil
}

func (bme *BME688) Pressure() (float64, error) {
	if !bme.running {
		return 0, bme688.ErrNotConnected
	}
	return bme.pressure, nil
}

func (bme *BME688) Humidity() (float64, error) {
	if !bme.running {
		return 0, bme688.ErrNotConnected
	}
	return bme.humidity, nil
}
