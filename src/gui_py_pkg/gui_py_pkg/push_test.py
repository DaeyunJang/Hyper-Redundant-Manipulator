import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton
from PyQt5.QtCore import QTimer

class MyWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setGeometry(300, 300, 300, 200)
        self.setWindowTitle('Long Press Example')

        self.button = QPushButton('Press and Hold', self)
        self.button.setGeometry(50, 50, 200, 100)

        self.button.pressed.connect(self.startLongPress)
        self.button.released.connect(self.stopLongPress)

        self.longPressDuration = 500  # milliseconds
        self.shortPressInterval = 100  # milliseconds

        self.longPressTimer = QTimer()
        self.longPressTimer.timeout.connect(self.handleLongPress)

    def startLongPress(self):
        self.longPressTimer.start(self.longPressDuration)

    def stopLongPress(self):
        self.longPressTimer.stop()

    def handleLongPress(self):
        print("Long press detected!")
        self.longPressTimer.setInterval(self.shortPressInterval)

def main(args=None):
    app = QApplication(sys.argv)
    widget = MyWidget()
    widget.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()