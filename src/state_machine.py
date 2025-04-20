#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int8
from mi_robot.srv import BuscarLata, BuscarLataRequest, BuscarDescargar, BuscarDescargarRequest, EvadirObstaculo, EvadirObstaculoRequest

class RobotStateMachine:
    def __init__(self):
        rospy.init_node('robot_state_machine')
        self.estado = "busqueda_lata"
        self.pub_estado = rospy.Publisher('/estado_robot', String, queue_size=10)
        self.pub_modo = rospy.Publisher('/modo', Int8, queue_size=1)
        self.buscar_lata_client = rospy.ServiceProxy('buscar_lata', BuscarLata)
        self.buscar_descargar_client = rospy.ServiceProxy('buscar_descargar', BuscarDescargar)
        self.evadir_obstaculo_client = rospy.ServiceProxy('evadir_obstaculo', EvadirObstaculo)
        rospy.loginfo("Máquina de estados inicializada.")
        self.run()

    def cambiar_estado(self, nuevo_estado):
        rospy.loginfo(f"Cambiando de {self.estado} a {nuevo_estado}")
        self.estado = nuevo_estado
        self.pub_estado.publish(self.estado)
        modo_msg = Int8()
        if nuevo_estado == "busqueda_lata":
            modo_msg.data = 2
        elif nuevo_estado == "busqueda_deposito":
            modo_msg.data = 3
        elif nuevo_estado == "descarga":
            modo_msg.data = 4
        elif nuevo_estado == "evasion":
            modo_msg.data = 5 # Nuevo modo para la evasión (opcional)
        else:
            modo_msg.data = 0
        self.pub_modo.publish(modo_msg)

    def ejecutar_busqueda_lata(self):
        rospy.loginfo("Llamando al servicio de busqueda de lata...")
        try:
            request = BuscarLataRequest()
            response = self.buscar_lata_client(request)
            if response.lata_encontrada:
                rospy.loginfo(f"Lata encontrada en x={response.centro_x}. Cambiando a busqueda de deposito.")
                self.cambiar_estado("busqueda_deposito")
            else:
                rospy.logwarn("No se encontró la lata.")
                # Decide qué hacer si no se encuentra la lata
        except rospy.ServiceException as e:
            rospy.logerr("La llamada al servicio de busqueda de lata falló: %s", e)

    def ejecutar_descarga(self):
        rospy.loginfo("Llamando al servicio de busqueda y descarga...")
        try:
            request = BuscarDescargarRequest()
            response = self.buscar_descargar_client(request)
            if response.exito:
                rospy.loginfo("Servicio de descarga completado con éxito: %s", response.mensaje)
                self.cambiar_estado("busqueda_lata")
            else:
                rospy.logwarn("Fallo en el servicio de descarga: %s", response.mensaje)
                self.cambiar_estado("busqueda_lata")
        except rospy.ServiceException as e:
            rospy.logerr("La llamada al servicio de descarga falló: %s", e)

    def ejecutar_evasion(self):
        rospy.loginfo("Llamando al servicio de evasión de obstáculos...")
        try:
            request = EvadirObstaculoRequest()
            response = self.evadir_obstaculo_client(request)
            if response.obstaculo_evadido:
                rospy.loginfo("Obstáculo evadido: %s", response.mensaje)
                self.cambiar_estado(self.previous_state) # Volver al estado anterior
            else:
                rospy.logwarn("No se pudo evadir el obstáculo o no se detectó ninguno: %s", response.mensaje)
                self.cambiar_estado(self.previous_state) # Volver al estado anterior
        except rospy.ServiceException as e:
            rospy.logerr("La llamada al servicio de evasión falló: %s", e)
            self.cambiar_estado(self.previous_state)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.estado == "busqueda_lata":
                self.ejecutar_busqueda_lata()
            elif self.estado == "busqueda_deposito":
                rospy.loginfo("Buscando deposito...")
                if rospy.get_time() % 15 > 7:
                    self.cambiar_estado("descarga")
            elif self.estado == "descarga":
                self.ejecutar_descarga()
            elif self.estado == "evasion":
                self.ejecutar_evasion()
            else:
                # Lógica de movimiento por defecto o espera
                twist = Twist()
                # Aquí podrías tener lógica para mover el robot hacia adelante
                # y verificar periódicamente si necesita evadir
                # Por ejemplo, podrías llamar al servicio de evasión cada cierto tiempo
                # o basándote en otros sensores.
                # Si detectas una necesidad de evadir, cambia el estado a "evasion".
                if self.obstacle_detected_externally(): # Función simulada para detección externa
                    self.previous_state = self.estado
                    self.cambiar_estado("evasion")
                self.cmd_vel_pub.publish(twist)

            rate.sleep()

    def obstacle_detected_externally(self):
        # Esto es solo un ejemplo. Deberías implementar tu propia lógica
        # para determinar cuándo llamar al servicio de evasión.
        # Podría basarse en la lectura directa de sensores en este nodo
        # o en la información de otros nodos.
        return False # Por defecto, no se detecta externamente

if __name__ == '__main__':
    try:
        robot_sm = RobotStateMachine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass