from rosboard_drv.rosboard_drv import RosBoardDrv
import time

def main():
    driver = RosBoardDrv(RosBoardDrv.CARTYPE_X3_PLUS, com="/dev/ttyUSB1",debug=True, logpath="./logs/")
    driver.create_receive_threading()
    driver.set_uart_servo_angle(4,120)
    print(driver.get_uart_servo_angle(4))
    time.sleep(1)

    print("ya acabo el programa")

if __name__ == "__main__":
    main()
