import rospy
from gazebo_msgs.srv import GetModelState
import matplotlib.pyplot as plt


class PlotCoordinates:
    def __init__(self):
        self.eboat_positions = []
        self.waypoint_positions = []

    def save_waypoint_coordinates(self):
        # Espera pelo serviço de pegar estado do modelo ficar disponível
        rospy.wait_for_service('/gazebo/get_model_state')
        get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # Define os nomes dos modelos e do referencial
        waypoint_name = 'wayPointMarker'
        reference_frame = 'world'

        # Pega o estado atual dos modelos
        waypoint_state = get_model_srv(waypoint_name, reference_frame)


        sim_time = rospy.Time.now()
        seconds = sim_time.secs

        # Adiciona as posições atuais às listas de posições
        self.waypoint_positions.append((waypoint_state.pose.position.x, waypoint_state.pose.position.y, seconds))

    def save_eboat_coordinates(self):
        # Espera pelo serviço de pegar estado do modelo ficar disponível
        rospy.wait_for_service('/gazebo/get_model_state')
        get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        # Define os nomes dos modelos e do referencial
        eboat_name = 'eboat'
        reference_frame = 'world'

        # Pega o estado atual dos modelos
        eboat_state = get_model_srv(eboat_name, reference_frame)

        # Adiciona as posições atuais às listas de posições
        self.eboat_positions.append((eboat_state.pose.position.x, eboat_state.pose.position.y))


    def plot_coordinates(self):
        # Configura o plot
        fig, ax = plt.subplots()        
        ax.set_xlim(-400, 400)
        ax.set_ylim(-400, 400)
        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        sim_time = rospy.Time.now()
        seconds = sim_time.secs
        ax.set_title('Trajetória do Eboat. Total time: {} seconds'.format(seconds))
        ax.plot(*zip(*self.eboat_positions), label='Eboat', linestyle='-')
        for i, (x, y, t) in enumerate(self.waypoint_positions):
            print(i+1, x, y, t)
            ax.plot(x, y, label=f'{i+1}°| t: {t}', linestyle='', marker='o')

        ax.legend()
        plt.draw()
        plt.pause(0.1)
        input("Pressione Enter para continuar...")
        
