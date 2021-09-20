// Package softrobotics implements the vacuum gripper from Soft Robotics.
package softrobotics

import (
	"context"
	"time"

	"github.com/go-errors/errors"

	"go.viam.com/utils"

	"go.viam.com/core/board"
	"go.viam.com/core/config"
	"go.viam.com/core/gripper"
	"go.viam.com/core/registry"
	"go.viam.com/core/robot"

	"github.com/edaniels/golog"
	"go.uber.org/multierr"
)

func init() {
	registry.RegisterGripper("softrobotics", registry.Gripper{Constructor: func(ctx context.Context, r robot.Robot, config config.Component, logger golog.Logger) (gripper.Gripper, error) {
		b, ok := r.BoardByName("local")
		if !ok {
			return nil, errors.New("softrobotics gripper requires a board called local")
		}
		return NewGripper(ctx, b, config, logger)
	}})
}

// Gripper TODO
//
// open is 5
// close is 6
type Gripper struct {
	theBoard board.Board

	psi board.AnalogReader

	pinOpen, pinClose, pinPower string

	logger golog.Logger
}

// NewGripper TODO
func NewGripper(ctx context.Context, b board.Board, config config.Component, logger golog.Logger) (*Gripper, error) {
	psi, ok := b.AnalogReaderByName("psi")
	if !ok {
		return nil, errors.New("failed to find analog reader 'psi'")
	}
	theGripper := &Gripper{
		theBoard: b,
		psi:      psi,
		pinOpen:  config.Attributes.String("open"),
		pinClose: config.Attributes.String("close"),
		pinPower: config.Attributes.String("power"),
		logger:   logger,
	}

	if theGripper.psi == nil {
		return nil, errors.New("no psi analog reader")
	}

	if theGripper.pinOpen == "" || theGripper.pinClose == "" || theGripper.pinPower == "" {
		return nil, errors.New("need pins for open, close, power")
	}

	return theGripper, nil
}

// Stop TODO
func (g *Gripper) Stop(ctx context.Context) error {
	return multierr.Combine(
		g.theBoard.GPIOSet(ctx, g.pinOpen, false),
		g.theBoard.GPIOSet(ctx, g.pinClose, false),
		g.theBoard.GPIOSet(ctx, g.pinPower, false),
	)
}

// Open TODO
func (g *Gripper) Open(ctx context.Context) error {
	err := multierr.Combine(
		g.theBoard.GPIOSet(ctx, g.pinOpen, true),
		g.theBoard.GPIOSet(ctx, g.pinPower, true),
	)
	if err != nil {
		return err
	}

	for {
		if !utils.SelectContextOrWait(ctx, 10*time.Millisecond) {
			return ctx.Err()
		} // REMOVE

		val, err := g.psi.Read(ctx)
		if err != nil {
			return multierr.Combine(err, g.Stop(ctx))
		}

		if val > 500 {
			break
		}

		if !utils.SelectContextOrWait(ctx, 10*time.Millisecond) {
			return ctx.Err()
		}
	}

	return g.Stop(ctx)
}

// Grab TODO
func (g *Gripper) Grab(ctx context.Context) (bool, error) {
	err := multierr.Combine(
		g.theBoard.GPIOSet(ctx, g.pinClose, true),
		g.theBoard.GPIOSet(ctx, g.pinPower, true),
	)
	if err != nil {
		return false, err
	}

	for {

		if !utils.SelectContextOrWait(ctx, 100*time.Millisecond) {
			return false, ctx.Err()
		} // REMOVE

		val, err := g.psi.Read(ctx)
		if err != nil {
			return false, multierr.Combine(err, g.Stop(ctx))
		}

		if val <= 200 {
			break
		}

		if !utils.SelectContextOrWait(ctx, 10*time.Millisecond) {
			return false, ctx.Err()
		}
	}

	return false, g.Stop(ctx)

}
