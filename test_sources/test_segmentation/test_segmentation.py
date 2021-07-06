import cv2
import numpy
import signal
import sys
import time
import yarp


class Module:

    def __init__(self):
        """Constructor."""

        # Parameters
        self.fps = 5

        # Open yarp input/output ports
        network = yarp.Network
        network.init()
        if not network.checkNetwork():
            print('YARP network not available. Please check your configuration.')
            sys.exit(1)

        self.rgb_in_buffer = None
        self.rgb_in = yarp.ImageRgb()
        self.rgb_in_port = yarp.BufferedPortImageRgb()
        self.rgb_in_port.open('/roft-yarp/test_segmentation/rgb:i')

        self.mask_out_buffer = None
        self.mask_out = yarp.ImageMono()
        self.mask_out_port = yarp.Port()
        self.mask_out_port.open('/roft-yarp/test_segmentation/mask:o')

        # Set SIGINT handler
        signal.signal(signal.SIGINT, self.sigint_handler)


    def sigint_handler(self, signal_number, frame):
        """SIGINT handler."""

        # Ignore additional signals to allow a proper cleanup
        signal.signal(signal_number, signal.SIG_IGN)

        # Close ports
        self.rgb_in_port.close()

        if self.mask_out_port.isOpen():
            self.mask_out_port.close()

        # Close
        sys.exit(0)


    def run(self):
        """Module main loop."""

        # Stream images
        while True:
            rgb = self.rgb_in_port.read(False)
            if not rgb:
                time.sleep(0.1)
                continue

            t0 = time.time()

            # Buffers initialization
            self.rgb_buffer = bytearray(numpy.zeros((rgb.height(), rgb.width(), 3), dtype = numpy.uint8))
            self.rgb_in.resize(rgb.width(), rgb.height())
            self.rgb_in.setExternal(self.rgb_buffer, rgb.width(), rgb.height())
            self.rgb_in.copy(rgb)

            # Access input as numpy array
            rgb_numpy = numpy.frombuffer(self.rgb_buffer, dtype=numpy.uint8).reshape(rgb.height(), rgb.width(), 3)
            gray = cv2.cvtColor(rgb_numpy, cv2.COLOR_BGR2GRAY)

            # Find the mask
            mask = numpy.zeros((rgb.height(), rgb.width()), dtype = numpy.uint8)
            mask[gray > 0] = 255

            # Send over output port
            self.mask_out_buffer = bytearray(mask.astype(numpy.uint8))
            self.mask_out.setExternal(self.mask_out_buffer, mask.shape[1], mask.shape[0])

            # Enforce fps
            elapsed = time.time() - t0
            wait_for = (1.0 / self.fps) - elapsed
            if (wait_for > 0):
                time.sleep(wait_for)

            self.mask_out_port.write(self.mask_out)


def main():
    module = Module()
    module.run()

if __name__ == '__main__':
    main()
