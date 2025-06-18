from rosboard_drv.rosboard_drv import RosBoardDrv
import time

def main():
    driver = RosBoardDrv(RosBoardDrv.CARTYPE_X3_PLUS, com="/dev/ttyUSB1",debug=True, logpath="./logs/")
    driver.create_receive_threading()
    for i in range(6):

            print(f"{i+1}={driver.get_uart_servo_angle(i+1)}")
        #driver.set_uart_servo_angle(3,60)
    #print(driver.get_uart_servo_angle(3))
    time.sleep(1)

    print("ya acabo el programa")

if __name__ == "__main__":
    main()
