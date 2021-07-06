import copy
import numpy
import os
import pyquaternion
import signal
import sys
import threading
import time
import trimesh
import yarp

# Force rendering with EGL
os.environ['PYOPENGL_PLATFORM'] = 'egl'
import pyrender


class Module():

    def __init__(self):
        """Constructor."""

        # Parameters
        self.fps = 30
        cam_w = 1280
        cam_h = 720
        cam_fx = 911.1308
        cam_fy = 911.1308
        cam_cx = 639.7758
        cam_cy = 357.4621

        # Open YARP output ports
        network = yarp.Network
        network.init()
        if not network.checkNetwork():
            print('YARP network not available. Please check your configuration.')
            sys.exit(1)

        self.rgb_out_buffer = None
        self.rgb_out = yarp.ImageRgb()
        self.rgb_out_port = yarp.Port()
        self.rgb_out_port.open('/roft-yarp/test_camera/rgb:o')

        self.depth_out_buffer = None
        self.depth_out = yarp.ImageFloat()
        self.depth_out_port = yarp.Port()
        self.depth_out_port.open('/roft-yarp/test_camera/depth:o')

        # Open YARP RPC server
        self.rpc_port = yarp.RpcServer()
        self.rpc_port.open('/roft-yarp/test_camera/rpc:i')
        self.rpc_thread = threading.Thread(target = self.respond)
        self.rpc_thread.daemon = True
        self.rpc_thread.start()

        # Parameters handled by the RPC server
        self.motion_enabled = False
        self.close_module = False
        self.rpc_lock = threading.Lock()

        # Load the mesh
        trimesh_mesh = trimesh.load('./test_sources/test_camera/mesh/obj_000005.obj')
        mesh = pyrender.Mesh.from_trimesh(trimesh_mesh)

        # Create the scene
        self.scene = pyrender.Scene(bg_color = [0.0, 0.0, 0.0])

        # Insert the object
        self.object_time = 0.0
        self.last_time = None
        self.object_pose_zero = pyquaternion.Quaternion(axis = (1.0, -1.0, 1.0), degrees = 200.0).transformation_matrix
        self.object_pose_zero[0:3, 3] = [0.0, 0.0, 0.5]
        self.object_node = pyrender.Node(mesh = mesh, matrix = self.object_pose_zero)
        self.scene.add_node(self.object_node)

        # Create and insert the camera
        camera = pyrender.IntrinsicsCamera(fx = cam_fx, fy = cam_fy, cx = cam_cx, cy = cam_cy)
        camera_gl_to_robotics = pyquaternion.Quaternion(axis = (1.0, 0.0, 0.0), degrees = 180.0).transformation_matrix
        self.scene.add(camera, pose = camera_gl_to_robotics)

        # Create and insert the light
        light = pyrender.PointLight(color = numpy.ones(3), intensity = 2.5)
        self.scene.add(light)

        # Initialize renderer
        self.renderer = pyrender.OffscreenRenderer(cam_w, cam_h)

        # Set SIGINT handler
        signal.signal(signal.SIGINT, self.sigint_handler)


    def cleanup(self):
        """Cleanup module before closing."""

        # Close ports
        if self.rgb_out_port.isOpen():
            self.rgb_out_port.close()

        if self.depth_out_port.isOpen():
            self.depth_out_port.close()

        self.rpc_port.close()


    def sigint_handler(self, signal_number, frame):
        """SIGINT handler."""

        # Ignore additional signals to allow a proper cleanup
        signal.signal(signal_number, signal.SIG_IGN)

        # Cleanup
        self.cleanup()

        # Close
        sys.exit(0)


    def run(self):
        """Module main loop."""

        # Stream images
        while True:

            self.rpc_lock.acquire()
            close_module = self.close_module
            motion_enabled = self.motion_enabled
            self.rpc_lock.release()

            if motion_enabled:
                if self.last_time:
                    self.object_time += time.time() - self.last_time

            if close_module:
                self.cleanup()

                return

            t0 = time.time()

            # Set new pose
            freq = 0.4
            amplitude_position = 0.2
            amplitude_angle = 60.0
            position_delta = [amplitude_position * numpy.sin(2 * numpy.pi * freq * self.object_time), 0.0, 0.0]
            rotation_delta = pyquaternion.Quaternion(axis = (0.0, 0.0, 1.0), degrees = amplitude_angle * numpy.sin(2 * numpy.pi * freq * self.object_time)).rotation_matrix

            self.object_pose_delta = copy.copy(self.object_pose_zero)
            self.object_pose_delta[0:3, 3] += position_delta
            self.object_pose_delta[0:3, 0:3] = rotation_delta.dot(self.object_pose_zero[0:3, 0:3])

            self.scene.set_pose(self.object_node, self.object_pose_delta)

            # Render
            render_rgb, render_depth = self.renderer.render(self.scene)

            # Send over output ports
            self.rgb_out_buffer = bytearray(render_rgb.astype(numpy.uint8))
            self.rgb_out.setExternal(self.rgb_out_buffer, render_rgb.shape[1], render_rgb.shape[0])
            self.rgb_out_port.write(self.rgb_out)

            self.depth_out_buffer = bytearray(render_depth.astype(numpy.float32))
            self.depth_out.setExternal(self.depth_out_buffer, render_depth.shape[1], render_depth.shape[0])
            self.depth_out_port.write(self.depth_out)

            # Enforce fps
            elapsed = time.time() - t0
            wait_for = (1.0 / self.fps) - elapsed
            self.last_time = time.time()
            if (wait_for > 0):
                time.sleep(wait_for)


    def respond(self):
        """Respond to RPC requests."""

        while True:
            request = yarp.Bottle()
            self.rpc_port.read(request, True)
            request_string = request.get(0).toString()

            reply = yarp.Bottle()

            if request_string == "start":
                self.rpc_lock.acquire()
                self.motion_enabled = True
                self.rpc_lock.release()

                reply.addString('Ack.')

            elif request_string == "stop":
                self.rpc_lock.acquire()
                self.motion_enabled = False
                self.rpc_lock.release()

                reply.addString('Ack.')

            elif request_string == "quit":
                self.rpc_lock.acquire()
                self.close_module = True
                self.rpc_lock.release()

                reply.addString('Ack.')

            else:
                reply.addVocab(yarp.encode('many'))
                reply.addString('Command not recognized.')
                reply.addString('List of commands: ')
                reply.addString('- start: start the object motion.')
                reply.addString('- stop: stop the object motion.')
                reply.addString('- quit: close the module.')

            self.rpc_port.reply(reply)

            if request_string == "quit":
                break



def main():
    module = Module()
    module.run()

if __name__ == '__main__':
    main()
