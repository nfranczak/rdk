// Package x264 contains the x264 video codec.
package x264

import (
	"context"
	"errors"
	"image"

	"github.com/pion/mediadevices/pkg/codec"
	"github.com/pion/mediadevices/pkg/codec/x264"
	"github.com/pion/mediadevices/pkg/prop"

	ourcodec "go.viam.com/rdk/gostream/codec"
	"go.viam.com/rdk/logging"
)

type encoder struct {
	codec  codec.ReadCloser
	img    image.Image
	logger logging.Logger
}

// NewEncoder returns an x264 encoder that can encode images of the given width and height. It will
// also ensure that it produces key frames at the given interval.
func NewEncoder(width, height, keyFrameInterval int, logger logging.Logger) (ourcodec.VideoEncoder, error) {
	// Check to make sure dimensions are even.
	if width%2 != 0 || height%2 != 0 {
		return nil, errors.New("x264 encoder does not support odd dimensions. " +
			"Please provide frames with even dimensions for width and height")
	}

	enc := &encoder{logger: logger}

	var builder codec.VideoEncoderBuilder
	params, err := x264.NewParams()
	if err != nil {
		return nil, err
	}
	builder = &params
	params.KeyFrameInterval = keyFrameInterval
	params.BitRate = calcBitrateFromResolution(width, height, float32(params.KeyFrameInterval))

	codec, err := builder.BuildVideoEncoder(enc, prop.Media{
		Video: prop.Video{
			Width:  width,
			Height: height,
		},
	})
	if err != nil {
		return nil, err
	}
	enc.codec = codec

	return enc, nil
}

// Read returns an image for codec to process.
func (v *encoder) Read() (img image.Image, release func(), err error) {
	return v.img, nil, nil
}

// Encode asks the codec to process the given image.
func (v *encoder) Encode(_ context.Context, img image.Image) ([]byte, error) {
	v.img = img
	data, release, err := v.codec.Read()
	dataCopy := make([]byte, len(data))
	copy(dataCopy, data)
	release()
	return dataCopy, err
}

// Close closes the encoder.
func (v *encoder) Close() error {
	return v.codec.Close()
}
