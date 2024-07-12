// Package bme688 provides a driver for Bosch's BME688 digital temperature & pressure sensor.
// The datasheet can be found here: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme688-ds000.pdf
package bme688

const Address byte = 0x77 // default I2C address

const (
	RegChipId    byte = 0xD0 // useful for checking the connection
	RegCali      byte = 0    // pressure & temperature compensation calibration coefficients, 0x8E thru 0xA0 for pressure, 0x8A 0x8B 0x8C 0xE9 0xEA for temperature
	RegPressXLSB byte = 0x21
	RegPressLSB  byte = 0x20
	RegPressMSB  byte = 0x1F
	RegTempXLSB  byte = 0x24
	RegTempLSB   byte = 0x23
	RegTempMSB   byte = 0x22
	RegPwrCtrl   byte = 0x74 // 1:0 measurement mode & pressure/temperature sensor power register
	RegOSR       byte = 0x74 // oversampling settings register
	RegStat      byte = 0x1D // sensor status register
	RegIIR       byte = 0x75
)

const (
	ChipId byte = 0x61 // correct response if reading from chip id register
)

// The difference between forced and parallel mode is the bme688 goes to sleep after taking a measurement in forced mode.
// Set it to forced if you intend to take measurements sporadically and want to save power. The driver will handle
// waking the sensor up when the sensor is in forced mode.
const (
	Parallel Mode = 0x02
	Forced   Mode = 0x01
	Sleep    Mode = 0x00
)

// Increasing sampling rate increases precision but also the wait time for measurements. The datasheet has a table of
// suggested values for oversampling, output data rates, and iir filter coefficients by use case.
const (
	Skipped Oversampling = iota
	Sampling1X
	Sampling2X
	Sampling4X
	Sampling8X
	Sampling16X
)

// IIR filter coefficients, higher values means steadier measurements but slower reaction times
const (
	Coeff0 FilterCoefficient = iota
	Coeff1
	Coeff3
	Coeff7
	Coeff15
	Coeff31
	Coeff63
	Coeff127
)
