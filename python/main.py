import sct

class Robot:

    def __init__(self, path):

        # Init controller
        self.sct = sct.SCT(path)

        ### Add callback functions for each event
        # Sensors (uncontrollable events)
        self.sct.add_callback(self.sct.EV['EV_btnMove'], None, self._check_btnMove, None)
        self.sct.add_callback(self.sct.EV['EV_btnStop'], None, self._check_btnStop, None)
        # Actions (controllable events)
        self.sct.add_callback(self.sct.EV['EV_move'], self._callback_move, None, None)
        self.sct.add_callback(self.sct.EV['EV_stop'], self._callback_stop, None, None)


    def control_step(self):
        self.sct.run_step()


    def _check_btnMove(self, data):
        print('Checking if btnMove was pressed...')
        # Check btnMove was pressed
        # if buttun was pressed
            # return true
        # else
            # return false


    def _check_btnStop(self, data):
        print('Checking if btnStop was pressed...')
        # Check btnStop was pressed
        # if buttun was pressed
            # return true
        # else
            # return false

    def _callback_move(self, data):
        print('Moving')
        # Make the robot move


    def _callback_stop(self, data):
        print('Stopped')
        # Make the robot stop


if __name__ == '__main__':
    robot = Robot('controller.yaml')
    print('loaded')

    while True:
        robot.control_step()
