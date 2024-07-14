package bme688

/*
taken from: https://github.com/tinygo-org/drivers/blob/release/bme688/bme688.go
and converted to use embed
*/
import (
	"errors"

	"github.com/kidoman/embd"
)

var (
	errConfigWrite  = errors.New("bme688: failed to configure sensor, check connection")
	errConfig       = errors.New("bme688: there is a problem with the configuration, try reducing ODR")
	errCaliRead     = errors.New("bme688: failed to read calibration coefficient register")
	errSoftReset    = errors.New("bme688: failed to perform a soft reset")
	ErrNotConnected = errors.New("bme688: not connected")
)

type Oversampling byte
type Mode byte
type OutputDataRate byte
type FilterCoefficient byte
type Config struct {
	Pressure    Oversampling
	Temperature Oversampling
	Humidity    Oversampling
	Mode        Mode
	ODR         OutputDataRate
	IIR         FilterCoefficient
}

// BME688 wraps the I2C connection and configuration values for the BME688
type BME688 struct {
	Bus     *embd.I2CBus
	Address uint8
	cali    calibrationCoefficients
	Config  Config
}

type calibrationCoefficients struct {
	// Temperature compensation
	t1 uint16
	t2 int16
	t3 int8

	// Pressure compensation
	p1  uint16
	p2  int16
	p3  int8
	p4  int16
	p5  int16
	p6  int8
	p7  int8
	p8  int16
	p9  int16
	p10 uint8

	// Humidty compensation
	h1 uint16
	h2 uint16
	h3 int8
	h4 int8
	h5 int8
	h6 uint8
	h7 int8
}

func (d *BME688) Configure(config Config) (err error) {
	d.Config = config

	if d.Config == (Config{}) {
		d.Config.Mode = Parallel
	}

	// Configure the oversampling, output data rate, and iir filter coefficient settings
	err1 := d.writeRegister(RegOSR, byte(d.Config.Pressure<<2|d.Config.Temperature<<5)|byte(d.Config.Mode))
	err2 := d.writeRegister(RegHum, byte(d.Config.Humidity))
	err3 := d.writeRegister(RegIIR, byte(d.Config.IIR<<2))

	if err1 != nil || err2 != nil || err3 != nil {
		return errConfigWrite
	}

	// Reading the builtin calibration coefficients and parsing them per the datasheet. The compensation formula given
	// in the datasheet is implemented in floating point
	buffer1, err := d.readRegister(RegCal1, 14) // starts at 0xE1
	if err != nil {
		return errCaliRead
	}
	buffer2, err := d.readRegister(RegCal2, 23) // starts at 0x8A
	if err != nil {
		return errCaliRead
	}

	d.cali.t1 = uint16(buffer1[9])<<8 | uint16(buffer1[8]) // 0xEA (MSB), 0xE9 (LSB)
	d.cali.t2 = int16(buffer2[1])<<8 | int16(buffer2[0])   // 0x8B (MSB), 0x8A (LSB)
	d.cali.t3 = int8(buffer2[2])                           // 0x8C

	d.cali.p1 = uint16(buffer2[5])<<8 | uint16(buffer2[4]) // 0x8F (MSB), 0x 08E(LSB)
	d.cali.p2 = int16(buffer2[7])<<8 | int16(buffer2[6])   // 0x91 (MSB), 0x90 (LSB)
	d.cali.p3 = int8(buffer2[8])                           // 0x92
	d.cali.p4 = int16(buffer2[11])<<8 | int16(buffer2[10]) // 0x95 (MSB), 0x94 (LSB)
	d.cali.p5 = int16(buffer2[13])<<8 | int16(buffer2[12]) // 0x97 (MSB), 0x96 (LSB)
	d.cali.p6 = int8(buffer2[15])                          // 0x99
	d.cali.p7 = int8(buffer2[14])                          // 0x98
	d.cali.p8 = int16(buffer2[19])<<8 | int16(buffer2[18]) // 0x9D (MSB), 0x9C (LSB)
	d.cali.p9 = int16(buffer2[21])<<8 | int16(buffer2[20]) // 0x9F (MSB), 0x9E (LSB)
	d.cali.p10 = uint8(buffer2[22])                        // 0xA0

	d.cali.h1 = uint16(buffer1[2])<<4 | (uint16(buffer1[1]) & 0x0F) // 0xE3 (MSB), 0xE2<3:0> (LSB)
	d.cali.h2 = uint16(buffer1[0])<<4 | uint16(buffer1[1])>>4       // 0xE1 (MSB), 0xE2<7:4> (LSB)
	d.cali.h3 = int8(buffer1[3])                                    // 0xE4
	d.cali.h4 = int8(buffer1[4])                                    // 0xE5
	d.cali.h5 = int8(buffer1[5])                                    // 0xE6
	d.cali.h6 = uint8(buffer1[6])                                   // 0xE7
	d.cali.h7 = int8(buffer1[7])                                    // 0xE8

	return nil
}
func (d *BME688) tlinCompensate() (int64, error) {
	rawTemp, err := d.readTempPressData(RegTempMSB, RegTempLSB, RegTempXLSB)
	if err != nil {
		return 0, err
	}

	// pulled from C driver: https://github.com/BoschSensortec/BME3-Sensor-API/blob/master/bme3.c
	partialData1 := (int64(rawTemp) >> 3) - (int64(d.cali.t1) << 1)
	partialData2 := (partialData1 * int64(d.cali.t2)) >> 11
	partialData3 := ((((partialData1 >> 1) * (partialData1 >> 1)) >> 12) * (int64(d.cali.t3) << 4)) >> 14
	return partialData2 + partialData3, nil

}
func (d *BME688) ReadTemperature() (float64, error) {

	tlin, err := d.tlinCompensate()
	if err != nil {
		return 0, err
	}

	temp := ((tlin * 5) + 128) >> 8
	return float64(temp), nil
}
func (d *BME688) ReadPressure() (float64, error) {

	tlin, err := d.tlinCompensate()
	if err != nil {
		return 0, err
	}
	rawPress, err := d.readTempPressData(RegPressMSB, RegPressLSB, RegPressXLSB)
	if err != nil {
		return 0, err
	}

	// code pulled from bme688 C driver: https://github.com/BoschSensortec/BME3-Sensor-API/blob/master/bme3.c
	partialData1 := (tlin >> 1) - 64000
	partialData2 := ((((partialData1 >> 2) * (partialData1 >> 2)) >> 11) * int64(d.cali.p6)) >> 2
	partialData2 = partialData2 + ((partialData1 * int64(d.cali.p5)) << 1)
	partialData2 = (partialData2 >> 2) + (int64(d.cali.p4) << 16)
	partialData1 = (((((partialData1 >> 2) * (partialData1 >> 2)) >> 13) * (int64(d.cali.p3) << 5)) >> 3) + ((int64(d.cali.p2) * partialData1) >> 1)
	partialData1 = partialData1 >> 18
	partialData1 = ((32768 + partialData1) * int64(d.cali.p1)) >> 15
	compPress := 1048576 - int64(rawPress)
	compPress = (compPress - (partialData2 >> 12)) * 3125
	if compPress >= (1 << 30) {
		compPress = (compPress / partialData1) << 1
	} else {
		compPress = (compPress << 1) / partialData1
	}
	partialData1 = (int64(d.cali.p9) * (((compPress >> 3) * (compPress >> 3)) >> 13)) >> 12
	partialData2 = ((compPress >> 2) * int64(d.cali.p8)) >> 13
	partialData3 := ((compPress >> 8) * (compPress >> 8) * (compPress >> 8) * int64(d.cali.p10)) >> 17
	compPress = compPress + ((partialData1 + partialData2 + partialData3 + (int64(d.cali.p7) << 7)) >> 4)
	return float64(compPress), nil
}
func (d *BME688) ReadHumidity() (float64, error) {
	tlin, err := d.tlinCompensate()
	if err != nil {
		return 0, err
	}
	rawHum, err := d.readHumidityData(RegHumMSB, RegHumLSB)
	if err != nil {
		return 0, err
	}

	tempScaled := ((tlin * 5) + 128) >> 8
	partialData1 := int64(rawHum) - (int64(d.cali.h1) << 4) - (((tempScaled * int64(d.cali.h3)) / 100) >> 1)
	partialData2 := (int64(d.cali.h2) * (((tempScaled * int64(d.cali.h4)) / 100) + (((tempScaled * ((tempScaled * int64(d.cali.h5)) / 100)) >> 6) / 100) + (1 << 14))) >> 10
	partialData3 := partialData1 * partialData2
	partialData4 := ((int64(d.cali.h6) << 7) + (tempScaled * int64(d.cali.h7) / 100)) >> 4
	partialData5 := ((partialData3 >> 14) * (partialData3 >> 14)) >> 10
	partialData6 := (partialData4 * partialData5) >> 1
	compHum := (((partialData3 + partialData6) >> 10) * 1000) >> 12
	if compHum > 100000 {
		compHum = 100000
	} else if compHum < 0 {
		compHum = 0
	}
	return float64(compHum) / 1000, nil
}
func (d *BME688) Connected() bool {
	data, err := d.readRegister(RegChipId, 1)
	return err == nil && (data[0] == ChipId) // returns true if i2c comm was good and response equals 0x50/0x60
}
func (d *BME688) SetMode(mode Mode) error {
	d.Config.Mode = mode
	return d.writeRegister(RegOSR, byte(d.Config.Pressure<<2|d.Config.Temperature<<5)|byte(d.Config.Mode))
}
func (d *BME688) readTempPressData(msb byte, lsb byte, xlsb byte) (data uint32, err error) {
	if !d.Connected() {
		return 0, ErrNotConnected
	}

	// put the sensor back into forced mode to get a reading, the sensor goes back to sleep after taking one read in
	// forced mode
	if d.Config.Mode != Parallel {
		err = d.SetMode(Forced)
		if err != nil {
			return
		}
	}

	bytes, err := d.readRegisters([]byte{msb, lsb, xlsb})
	if err != nil {
		return
	}
	data = uint32(bytes[0])<<12 | uint32(bytes[1])<<4 | uint32(bytes[2])>>4
	return
}

func (d *BME688) readHumidityData(msb byte, lsb byte) (data uint16, err error) {
	if !d.Connected() {
		return 0, ErrNotConnected
	}

	// put the sensor back into forced mode to get a reading, the sensor goes back to sleep after taking one read in
	// forced mode
	if d.Config.Mode != Parallel {
		err = d.SetMode(Forced)
		if err != nil {
			return
		}
	}

	bytes, err := d.readRegisters([]byte{msb, lsb})
	if err != nil {
		return
	}
	data = uint16(bytes[0])<<8 | uint16(bytes[1])
	return
}

func (d *BME688) readRegisters(register []byte) (data []byte, err error) {
	data = make([]byte, len(register))
	err = nil
	for i := 0; i < len(register); i++ {
		datum, _error := (*d.Bus).ReadByteFromReg(d.Address, register[i])
		data[i] = datum
		if _error != nil {
			err = _error
		}
	}
	return
}

func (d *BME688) readRegister(register byte, len int) (data []byte, err error) {
	data = make([]byte, len)
	err = (*d.Bus).ReadFromReg(d.Address, register, data)
	return
}

func (d *BME688) writeRegister(register byte, data byte) error {
	return (*d.Bus).WriteToReg(d.Address, register, []byte{data})
}
