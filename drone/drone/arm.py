from pymavlink import mavutil
import connect
import mode

mode_en = ['STABILIZE','GUIDED','ACRO','ALT_HOLD','LOITER','POSHOLD']



def arm(the_connection):
    mode_act = mode.get_mode(the_connection)
    

    if not check_if_arm(the_connection):
        if mode_act in mode_en:
            print("Armo il drone...")

            try:
                the_connection.mav.command_long_send(
                    the_connection.target_system,
                    the_connection.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,
                    1,   #arma il drone 
                    0, 0, 0, 0, 0, 0
                )
        
                msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=10 )
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("Drone armato!!")
                    
                else:
                    print("Errore, drone non armato")
                    
            except Exception as e:
                print(f"Errore durante l'armamento drone: {e}")
        else:
            print("Cambiare modalità di volo per armare il drone")
            return 
    else:
        print("Drone già armato!!")

def check_if_arm(the_connection):
    msg = the_connection.recv_match(type = 'HEARTBEAT',blocking = True)
    if msg:
        system_status = msg.system_status
        if system_status == mavutil.mavlink.MAV_STATE_ACTIVE:
            return True
    
    return False


if __name__ == "__main__":
    the_connection = connect.connect('udpin:127.0.0.1:5760',10)
    arm( the_connection )


