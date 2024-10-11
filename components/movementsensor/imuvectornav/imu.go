//go:build linux

// Package imuvectornav implements a component for a vectornav IMU.
package imuvectornav

import (
	"context"
	"encoding/binary"
	"sync"
	"time"

	"github.com/golang/geo/r3"
	geo "github.com/kellydunn/golang-geo"
	"github.com/pkg/errors"
	goutils "go.viam.com/utils"

	"go.viam.com/rdk/components/board/genericlinux/buses"
	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
)

var model = resource.DefaultModelFamily.WithModel("imu-vectornav")

// DefaultBaudRate is the value we assign if there is no value specified in the attributes.
const DefaultBaudRate = 9600

// Config is used for converting a vectornav IMU MovementSensor config attributes.
type Config struct {
	SPI      string `json:"spi_bus"`
	BaudRate *int   `json:"spi_baud_rate,omitempty"`
	Pfreq    *int   `json:"polling_freq_hz,omitempty"`
	GPIOPin  string `json:"chip_select_pin"` // this is the gpio number that we plugged it into
}

// Validate ensures all parts of the config are valid.
func (cfg *Config) Validate(path string) ([]string, error) {
	var deps []string
	if cfg.SPI == "" {
		return nil, resource.NewConfigValidationFieldRequiredError(path, "spi")
	}

	if cfg.GPIOPin == "" {
		return nil, resource.NewConfigValidationFieldRequiredError(path, "cs_pin (chip select pin)")
	}
	return deps, nil
}

func init() {
	resource.RegisterComponent(movementsensor.API, model, resource.Registration[movementsensor.MovementSensor, *Config]{
		Constructor: newVectorNav,
	})
}

type vectornav struct {
	resource.Named
	resource.AlwaysRebuild
	angularVelocity spatialmath.AngularVelocity
	acceleration    r3.Vector
	magnetometer    r3.Vector
	dV              r3.Vector
	dTheta          r3.Vector
	dt              float32
	orientation     spatialmath.EulerAngles

	mu      sync.Mutex
	spiMu   sync.Mutex
	polling uint

	workers   *goutils.StoppableWorkers
	bus       buses.SPI
	gpioPin   string
	baudRate  int
	logger    logging.Logger
	busClosed bool

	bdVX float64
	bdVY float64
	bdVZ float64
}

const (
	vectorNavSPIRead  uint = 1
	vectorNavSPIWrite uint = 2
	vectorNavSPITare  uint = 5
)

type vectornavRegister uint

const (
	modelNumber                  vectornavRegister = 1
	serialNumber                 vectornavRegister = 3
	firmwareVersion              vectornavRegister = 4
	deltaVDeltaTheta             vectornavRegister = 80
	deltaVDeltaThetaConfig       vectornavRegister = 82
	yawPitchRollMagAccGyro       vectornavRegister = 27
	acceleration                 vectornavRegister = 18
	referenceVectorConfiguration vectornavRegister = 83
	magAccRefVectors             vectornavRegister = 21
	accCompensationConfiguration vectornavRegister = 25
	vpeAccTunning                vectornavRegister = 38
)

// newVectorNav connect and set up a vectornav IMU over SPI.
// Will also compensate for acceleration and delta velocity bias over one second so be
// sure the IMU is still when calling this function.
func newVectorNav(
	ctx context.Context,
	deps resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (movementsensor.MovementSensor, error) {
	newConf, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}

	baudRate, pollFreq, gpioPin := configParsingAndSetup(ctx, logger, newConf)

	v := &vectornav{
		Named:     conf.ResourceName().AsNamed(),
		bus:       buses.NewSpiBus(newConf.SPI),
		logger:    logger,
		gpioPin:   gpioPin,
		baudRate:  baudRate,
		busClosed: false,
		polling:   uint(pollFreq),
	}

	err = v.registerReading(ctx, logger)
	if err != nil {
		return nil, err
	}

	err = v.imuConfig(ctx)
	if err != nil {
		return nil, err
	}

	err = v.biasCompensation(ctx, pollFreq)
	if err != nil {
		return nil, err
	}

	go v.pollingSetup(ctx, logger, pollFreq)

	return v, nil
}

func configParsingAndSetup(ctx context.Context, logger logging.Logger, cfg *Config) (int, int, string) {
	baudRate := *cfg.BaudRate
	if baudRate == 0 {
		logger.CInfof(ctx, "we are setting the baudRate to %d since no value was passed in", DefaultBaudRate)
		baudRate = DefaultBaudRate
	}

	pollFreq := 200
	if *cfg.Pfreq > 0 {
		pollFreq = *cfg.Pfreq
	}

	return baudRate, pollFreq, cfg.GPIOPin
}

func (vn *vectornav) registerReading(ctx context.Context, logger logging.Logger) error {
	mdl, err := vn.readRegisterSPI(ctx, modelNumber, 24)
	if err != nil {
		return err
	}
	sn, err := vn.readRegisterSPI(ctx, serialNumber, 4)
	if err != nil {
		return err
	}
	fwver, err := vn.readRegisterSPI(ctx, firmwareVersion, 4)
	if err != nil {
		return err
	}
	logger.CDebugf(ctx,
		"model detected %s sn %d %d.%d.%d.%d",
		string(mdl),
		binary.LittleEndian.Uint32(sn),
		fwver[0],
		fwver[1],
		fwver[2],
		fwver[3],
	)
	return nil
}

func (vn *vectornav) imuConfig(ctx context.Context) error {
	// set imu location to New York for the WGM model
	refvec := []byte{1, 1, 0, 0}
	refvec = append(refvec, utils.BytesFromUint32LE(1000)...)
	refvec = append(refvec, utils.BytesFromFloat32LE(2010.0)...)
	refvec = append(refvec, []byte{0, 0, 0, 0}...)
	refvec = append(refvec, utils.BytesFromFloat64LE(40.730610)...)
	refvec = append(refvec, utils.BytesFromFloat64LE(-73.935242)...)
	refvec = append(refvec, utils.BytesFromFloat64LE(10.0)...)
	err := vn.writeRegisterSPI(ctx, referenceVectorConfiguration, refvec)
	if err != nil {
		return errors.Wrap(err, "couldn't set reference vector")
	}
	// enforce acceleration tuinning and reduce "trust" in acceleration data
	accVpeTunning := []byte{}
	accVpeTunning = append(accVpeTunning, utils.BytesFromFloat32LE(3)...)
	accVpeTunning = append(accVpeTunning, utils.BytesFromFloat32LE(3)...)
	accVpeTunning = append(accVpeTunning, utils.BytesFromFloat32LE(3)...)
	accVpeTunning = append(accVpeTunning, utils.BytesFromFloat32LE(10)...)
	accVpeTunning = append(accVpeTunning, utils.BytesFromFloat32LE(10)...)
	accVpeTunning = append(accVpeTunning, utils.BytesFromFloat32LE(10)...)
	accVpeTunning = append(accVpeTunning, utils.BytesFromFloat32LE(10)...)
	accVpeTunning = append(accVpeTunning, utils.BytesFromFloat32LE(10)...)
	accVpeTunning = append(accVpeTunning, utils.BytesFromFloat32LE(10)...)

	err = vn.writeRegisterSPI(ctx, vpeAccTunning, accVpeTunning)
	if err != nil {
		return errors.Wrap(err, "couldn't set vpe adaptive tunning")
	}
	err = vn.writeRegisterSPI(ctx, deltaVDeltaThetaConfig, []byte{0, 0, 0, 0, 0, 0})
	if err != nil {
		return errors.Wrap(err, "couldn't configure deltaV register")
	}
	return nil
}

func (vn *vectornav) biasCompensation(ctx context.Context, pollFreq int) error {
	// tare the heading
	err := vn.vectornavTareSPI(ctx)
	if err != nil {
		return err
	}
	// compensate for acceleration bias due to misalignement
	err = vn.compensateAccelBias(ctx, uint(pollFreq))
	if err != nil {
		return err
	}
	// compensate for constant DV bias in mesurament
	err = vn.compensateDVBias(ctx, uint(pollFreq))
	if err != nil {
		return err
	}
	return nil
}

func (vn *vectornav) pollingSetup(ctx context.Context, logger logging.Logger, pollFreq int) {
	logger.CDebugf(ctx, "vecnav: will poll at %d Hz", pollFreq)
	waitCh := make(chan struct{})
	pollHertz := 1.0 / float64(pollFreq)
	vn.workers = goutils.NewBackgroundStoppableWorkers(func(cancelCtx context.Context) {
		timer := time.NewTicker(time.Duration(pollHertz * float64(time.Second)))
		defer timer.Stop()
		close(waitCh)
		for {
			select {
			case <-cancelCtx.Done():
				return
			default:
			}
			select {
			case <-cancelCtx.Done():
				return
			case <-timer.C:
				err := vn.getReadings(ctx)
				if err != nil {
					return
				}
			}
		}
	})
	<-waitCh
}

func (vn *vectornav) AngularVelocity(ctx context.Context, extra map[string]interface{}) (spatialmath.AngularVelocity, error) {
	vn.mu.Lock()
	defer vn.mu.Unlock()
	return vn.angularVelocity, nil
}

func (vn *vectornav) LinearAcceleration(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
	vn.mu.Lock()
	defer vn.mu.Unlock()
	return vn.acceleration, nil
}

func (vn *vectornav) Orientation(ctx context.Context, extra map[string]interface{}) (spatialmath.Orientation, error) {
	vn.mu.Lock()
	defer vn.mu.Unlock()
	return &vn.orientation, nil
}

func (vn *vectornav) CompassHeading(ctx context.Context, extra map[string]interface{}) (float64, error) {
	vn.mu.Lock()
	defer vn.mu.Unlock()
	return vn.orientation.Yaw, nil
}

func (vn *vectornav) LinearVelocity(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
	return r3.Vector{}, movementsensor.ErrMethodUnimplementedLinearVelocity
}

func (vn *vectornav) Position(ctx context.Context, extra map[string]interface{}) (*geo.Point, float64, error) {
	return nil, 0, movementsensor.ErrMethodUnimplementedPosition
}

func (vn *vectornav) Accuracy(ctx context.Context, extra map[string]interface{}) (*movementsensor.Accuracy, error) {
	// TODO:  RSDK-6389 check the vectornav's datasheet to determine what is best to return from the vector nav.
	// can be done in a seprate ticket from the one mentioned in this comment.
	return movementsensor.UnimplementedOptionalAccuracies(), nil
}

func (vn *vectornav) GetMagnetometer(ctx context.Context) (r3.Vector, error) {
	vn.mu.Lock()
	defer vn.mu.Unlock()
	return vn.magnetometer, nil
}

func (vn *vectornav) Properties(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
	return &movementsensor.Properties{
		AngularVelocitySupported:    true,
		OrientationSupported:        true,
		LinearAccelerationSupported: true,
	}, nil
}

func (vn *vectornav) Readings(ctx context.Context, extra map[string]interface{}) (map[string]interface{}, error) {
	return movementsensor.DefaultAPIReadings(ctx, vn, extra)
}

func (vn *vectornav) getReadings(ctx context.Context) error {
	out, err := vn.readRegisterSPI(ctx, yawPitchRollMagAccGyro, 48)
	if err != nil {
		return err
	}
	vn.mu.Lock()
	defer vn.mu.Unlock()
	vn.orientation.Yaw = utils.DegToRad(float64(utils.Float32FromBytesLE(out[0:4])))
	vn.orientation.Pitch = utils.DegToRad(float64(utils.Float32FromBytesLE(out[4:8])))
	vn.orientation.Roll = utils.DegToRad(float64(utils.Float32FromBytesLE(out[8:12])))
	// unit gauss
	vn.magnetometer.X = float64(utils.Float32FromBytesLE(out[12:16]))
	vn.magnetometer.Y = float64(utils.Float32FromBytesLE(out[16:20]))
	vn.magnetometer.Z = float64(utils.Float32FromBytesLE(out[20:24]))
	// unit m/s^2
	vn.acceleration.X = float64(utils.Float32FromBytesLE(out[24:28]))
	vn.acceleration.Y = float64(utils.Float32FromBytesLE(out[28:32]))
	vn.acceleration.Z = float64(utils.Float32FromBytesLE(out[32:36]))
	// unit rad/s
	vn.angularVelocity.X = utils.RadToDeg(float64(utils.Float32FromBytesLE(out[36:40])))
	vn.angularVelocity.Y = utils.RadToDeg(float64(utils.Float32FromBytesLE(out[40:44])))
	vn.angularVelocity.Z = utils.RadToDeg(float64(utils.Float32FromBytesLE(out[44:48])))
	dv, err := vn.readRegisterSPI(ctx, deltaVDeltaTheta, 28)
	if err != nil {
		return err
	}
	// unit deg/s
	vn.dTheta.X = float64(utils.Float32FromBytesLE(dv[4:8]))
	vn.dTheta.Y = float64(utils.Float32FromBytesLE(dv[8:12]))
	vn.dTheta.Z = float64(utils.Float32FromBytesLE(dv[12:16]))
	// unit m/s
	vn.dV.X = float64(utils.Float32FromBytesLE(dv[16:20])) - vn.bdVX
	vn.dV.Y = float64(utils.Float32FromBytesLE(dv[20:24])) - vn.bdVY
	vn.dV.Z = float64(utils.Float32FromBytesLE(dv[24:28])) - vn.bdVZ
	// unit s
	vn.dt = utils.Float32FromBytesLE(dv[0:4])
	return nil
}

func (vn *vectornav) readRegisterSPI(ctx context.Context, reg vectornavRegister, readLen uint) ([]byte, error) {
	vn.spiMu.Lock()
	defer vn.spiMu.Unlock()
	if vn.busClosed {
		return nil, errors.New("C=cannot read spi register the bus is closed")
	}
	hnd, err := vn.bus.OpenHandle()
	if err != nil {
		return nil, err
	}
	cmd := []byte{byte(vectorNavSPIRead), byte(reg), 0, 0}
	_, err = hnd.Xfer(ctx, uint(vn.baudRate), vn.gpioPin, 3, cmd)
	if err != nil {
		return nil, err
	}
	goutils.SelectContextOrWait(ctx, 110*time.Microsecond)
	cmd = make([]byte, readLen+4)
	out, err := hnd.Xfer(ctx, uint(vn.baudRate), vn.gpioPin, 3, cmd)
	if err != nil {
		return nil, err
	}
	if out[3] != 0 {
		return nil, errors.Errorf("vectornav read error returned %d baudRate was %d", out[3], vn.baudRate)
	}
	err = hnd.Close()
	if err != nil {
		return nil, err
	}
	goutils.SelectContextOrWait(ctx, 110*time.Microsecond)
	return out[4:], nil
}

func (vn *vectornav) writeRegisterSPI(ctx context.Context, reg vectornavRegister, data []byte) error {
	vn.spiMu.Lock()
	defer vn.spiMu.Unlock()
	if vn.busClosed {
		return errors.New("Cannot write spi register the bus is closed")
	}
	hnd, err := vn.bus.OpenHandle()
	if err != nil {
		return err
	}
	cmd := []byte{byte(vectorNavSPIWrite), byte(reg), 0, 0}
	cmd = append(cmd, data...)
	_, err = hnd.Xfer(ctx, uint(vn.baudRate), vn.gpioPin, 3, cmd)
	if err != nil {
		return err
	}
	goutils.SelectContextOrWait(ctx, 110*time.Microsecond)
	cmd = make([]byte, len(data)+4)
	out, err := hnd.Xfer(ctx, uint(vn.baudRate), vn.gpioPin, 3, cmd)
	if err != nil {
		return err
	}
	if out[3] != 0 {
		return errors.Errorf("vectornav write error returned %d", out[3])
	}
	err = hnd.Close()
	if err != nil {
		return err
	}
	goutils.SelectContextOrWait(ctx, 110*time.Microsecond)
	return nil
}

func (vn *vectornav) vectornavTareSPI(ctx context.Context) error {
	vn.spiMu.Lock()
	defer vn.spiMu.Unlock()
	if vn.busClosed {
		return errors.New("Cannot write spi register the bus is closed")
	}
	hnd, err := vn.bus.OpenHandle()
	if err != nil {
		return err
	}
	cmd := []byte{byte(vectorNavSPITare), 0, 0, 0}
	_, err = hnd.Xfer(ctx, uint(vn.baudRate), vn.gpioPin, 3, cmd)
	if err != nil {
		return err
	}
	goutils.SelectContextOrWait(ctx, 110*time.Microsecond)
	cmd = []byte{0, 0, 0, 0}
	out, err := hnd.Xfer(ctx, uint(vn.baudRate), vn.gpioPin, 3, cmd)
	if err != nil {
		return err
	}
	if out[3] != 0 {
		return errors.Errorf("vectornav write error returned %d", out[3])
	}
	err = hnd.Close()
	if err != nil {
		return err
	}
	goutils.SelectContextOrWait(ctx, 110*time.Microsecond)
	return nil
}

func (vn *vectornav) compensateAccelBias(ctx context.Context, smpSize uint) error {
	var msg []byte
	msg = append(msg, utils.BytesFromFloat32LE(1.0)...)
	msg = append(msg, utils.BytesFromFloat32LE(0.0)...)
	msg = append(msg, utils.BytesFromFloat32LE(0.0)...)

	msg = append(msg, utils.BytesFromFloat32LE(0.0)...)
	msg = append(msg, utils.BytesFromFloat32LE(1.0)...)
	msg = append(msg, utils.BytesFromFloat32LE(0.0)...)

	msg = append(msg, utils.BytesFromFloat32LE(0.0)...)
	msg = append(msg, utils.BytesFromFloat32LE(0.0)...)
	msg = append(msg, utils.BytesFromFloat32LE(1.0)...)

	msg = append(msg, utils.BytesFromFloat32LE(0.0)...)
	msg = append(msg, utils.BytesFromFloat32LE(0.0)...)
	msg = append(msg, utils.BytesFromFloat32LE(0.0)...)
	err := vn.writeRegisterSPI(ctx, accCompensationConfiguration, msg)
	if err != nil {
		return errors.Wrap(err, "couldn't write the acceleration compensation register")
	}
	mdlG, err := vn.readRegisterSPI(ctx, magAccRefVectors, 24)
	if err != nil {
		return errors.Wrap(err, "couldn't calculate acceleration bias")
	}
	accZ := utils.Float32FromBytesLE(mdlG[20:24])
	var accMX, accMY, accMZ float32
	for i := uint(0); i < smpSize; i++ {
		acc, err := vn.readRegisterSPI(ctx, acceleration, 12)
		if err != nil {
			return errors.Wrap(err, "error reading acceleration register during bias compensation")
		}
		accMX += utils.Float32FromBytesLE(acc[0:4])
		accMY += utils.Float32FromBytesLE(acc[4:8])
		accMZ += utils.Float32FromBytesLE(acc[8:12])
		if !goutils.SelectContextOrWait(ctx, 50*time.Microsecond) {
			return errors.New("error in context during acceleration compensation")
		}
	}
	accMX /= float32(smpSize)
	accMY /= float32(smpSize)
	accMZ /= float32(smpSize)
	msg = []byte{}
	msg = append(msg, utils.BytesFromFloat32LE(1.0)...)
	msg = append(msg, utils.BytesFromFloat32LE(0.0)...)
	msg = append(msg, utils.BytesFromFloat32LE(0.0)...)

	msg = append(msg, utils.BytesFromFloat32LE(0.0)...)
	msg = append(msg, utils.BytesFromFloat32LE(1.0)...)
	msg = append(msg, utils.BytesFromFloat32LE(0.0)...)

	msg = append(msg, utils.BytesFromFloat32LE(0.0)...)
	msg = append(msg, utils.BytesFromFloat32LE(0.0)...)
	msg = append(msg, utils.BytesFromFloat32LE(1.0)...)

	msg = append(msg, utils.BytesFromFloat32LE(accMX)...)
	msg = append(msg, utils.BytesFromFloat32LE(accMY)...)
	msg = append(msg, utils.BytesFromFloat32LE(accZ+accMZ)...)

	err = vn.writeRegisterSPI(ctx, accCompensationConfiguration, msg)
	if err != nil {
		return errors.Wrap(err, "could not write the acceleration register")
	}
	vn.logger.CInfof(ctx, "Acceleration compensated with %1.6f %1.6f %1.6f ref accZ %1.6f", accMX, accMY, accMZ, accZ)
	return nil
}

func (vn *vectornav) compensateDVBias(ctx context.Context, smpSize uint) error {
	var bX, bY, bZ float32
	_, err := vn.readRegisterSPI(ctx, deltaVDeltaTheta, 28)
	if err != nil {
		return errors.Wrap(err, "error reading dV register during bias compensation")
	}
	dt := 10 * time.Millisecond
	if vn.polling > 0 {
		s := 1.0 / float64(vn.polling)
		dt = time.Duration(s * float64(time.Second))
	}
	for j := uint(0); j < smpSize; j++ {
		if !goutils.SelectContextOrWait(ctx, dt) {
			return errors.New("error in context during Dv compensation")
		}
		dv, err := vn.readRegisterSPI(ctx, deltaVDeltaTheta, 28)
		if err != nil {
			return errors.Wrap(err, "error reading dV register during bias compensation")
		}
		bX += utils.Float32FromBytesLE(dv[16:20])
		bY += utils.Float32FromBytesLE(dv[20:24])
		bZ += utils.Float32FromBytesLE(dv[24:28])
	}
	vn.bdVX = float64(bX) / float64(smpSize)
	vn.bdVY = float64(bY) / float64(smpSize)
	vn.bdVZ = float64(bZ) / float64(smpSize)
	vn.logger.CInfof(ctx, "velocity bias compensated with %1.6f %1.6f %1.6f",
		vn.bdVX, vn.bdVY, vn.bdVZ)
	return nil
}

func (vn *vectornav) Close(ctx context.Context) error {
	vn.logger.CDebug(ctx, "closing vecnav imu")
	vn.busClosed = true
	vn.workers.Stop()
	vn.logger.CDebug(ctx, "closed vecnav imu")
	return nil
}
