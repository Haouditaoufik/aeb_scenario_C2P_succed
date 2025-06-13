import carla
import time
import math
import pygame
import sys
import os
import socket
import struct
from carla import Color
import matplotlib.pyplot as plt
from collections import deque

def get_speed(vehicle):
    velocity = vehicle.get_velocity()
    return math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

def get_obstacle_type(actor):
    if 'vehicle' in actor.type_id:
        return 'VEHICLE'
    elif 'walker' in actor.type_id:
        return 'PEDESTRIAN'
    elif 'bicycle' in actor.type_id:
        return 'BICYCLE'
    return 'UNKNOWN'

def draw_bounding_box(world, actor, color=Color(r=0, g=255, b=0)):
    debug = world.debug
    bb = actor.bounding_box
    transform = actor.get_transform()
    vertices = [
        carla.Location(x=-bb.extent.x, y=-bb.extent.y, z=-bb.extent.z),
        carla.Location(x=bb.extent.x, y=-bb.extent.y, z=-bb.extent.z),
        carla.Location(x=bb.extent.x, y=bb.extent.y, z=-bb.extent.z),
        carla.Location(x=-bb.extent.x, y=bb.extent.y, z=-bb.extent.z),
        carla.Location(x=-bb.extent.x, y=-bb.extent.y, z=bb.extent.z),
        carla.Location(x=bb.extent.x, y=-bb.extent.y, z=bb.extent.z),
        carla.Location(x=bb.extent.x, y=bb.extent.y, z=bb.extent.z),
        carla.Location(x=-bb.extent.x, y=bb.extent.y, z=bb.extent.z)
    ]
    vertices = [transform.transform(v) for v in vertices]
    edges = [(0,1), (1,2), (2,3), (3,0), (4,5), (5,6), (6,7), (7,4), (0,4), (1,5), (2,6), (3,7)]
    for edge in edges:
        debug.draw_line(vertices[edge[0]], vertices[edge[1]], thickness=0.1, color=color, life_time=0.1)

def setup_tcp_server(port=9001):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('localhost', port))
    sock.listen(1)
    print(f"En attente de connexion Simulink sur le port {port}...")
    connection, _ = sock.accept()
    connection.settimeout(0.1)
    print("Connexion Simulink établie!")
    return connection

def send_data(conn, data):
    try:
        values = [data['MIO_Distance'], data['MIO_Velocity'], data['Ego_Velocity']]
        for val in values:
            conn.sendall(struct.pack('d', float(val)))
        print(f"Données envoyées: {values}")
    except Exception as e:
        print(f"Erreur d'envoi: {e}")
        raise

def receive_data(conn):
    try:
        values = []
        for _ in range(4):
            try:
                data = conn.recv(8)
                if len(data) < 8:
                    print("Données incomplètes reçues, réessai...")
                    time.sleep(0.01)
                    continue
                values.append(struct.unpack('d', data)[0])
            except socket.timeout:
                print("Timeout en attente de données")
                return None
        result = {
            'egoCarStop': bool(round(values[0])),
            'FCW_Activate': bool(round(values[1])),
            'Deceleration': float(values[2]),
            'AEB_Status': bool(round(values[3]))
        }
        print(f"Données reçues: {result}")
        return result
    except Exception as e:
        print(f"Erreur de réception: {e}")
        return None

def draw_hud(screen, font, ego_speed, throttle, brake, ttc, collision_status,
             aeb_status, fcw_active, obstacle_type="NONE", pedestrian_visible=False):
    screen.fill((0, 0, 0))
    y_pos = 30
    lines = [
        ("AEB PEDESTRIAN TEST MONITOR", (255, 255, 255)),
        (f"SPEED: {ego_speed:.1f} km/h", (255, 255, 255)),
        (f"THROTTLE: {throttle:.2f}", (0, 255, 0) if throttle < 0.1 else (255, 255, 0)),
        (f"BRAKE: {brake:.2f}", (255, 0, 0) if brake > 0.1 else (255, 255, 255)),
        (f"TTC: {ttc:.2f} s", (255, 0, 0) if ttc < 2.0 else (255, 255, 0) if ttc < 5.0 else (0, 255, 0)),
        (f"AEB: {aeb_status}", (255, 0, 0) if aeb_status == "ACTIVE" else (0, 255, 0)),
        (f"FCW: {'ACTIVE' if fcw_active else 'INACTIVE'}", (255, 165, 0) if fcw_active else (255, 255, 255)),
        (f"OBSTACLE: {obstacle_type}", (255, 255, 0)),
        (f"PEDESTRIAN VISIBLE: {'YES' if pedestrian_visible else 'NO'}", (255, 255, 0) if pedestrian_visible else (255, 128, 128)),
        ("STATUS: COLLISION!" if collision_status else "STATUS: NO COLLISION",
         (255, 0, 0) if collision_status else (0, 255, 0)),
        ("CONNECTION: ACTIVE", (0, 255, 0))
    ]
    for line, color in lines:
        text = font.render(line, True, color)
        screen.blit(text, (10, y_pos))
        y_pos += 30
    pygame.display.flip()

def initialize_carla():
    print("Connexion au serveur CARLA...")
    client = carla.Client('localhost', 2000)
    client.set_timeout(15.0)
    print("Chargement de Town03...")
    world = client.load_world('Town03')
    time.sleep(2)
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)
    return client, world
  
def spawn_actors(world):
    try:
        print("Nettoyage des acteurs existants...")
        for actor in world.get_actors().filter('*vehicle*'):
            actor.destroy()
        for actor in world.get_actors().filter('*walker*'):
            actor.destroy()
        time.sleep(1)

        blueprint_library = world.get_blueprint_library()
        
        ego_spawn = carla.Transform(
            carla.Location(x=185.0, y=8.2, z=0.3),
            carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
        )
        
        pedestrian_spawn = carla.Transform(
            carla.Location(x=207.0, y=4.0, z=0.3),  
            carla.Rotation(pitch=0.0, yaw=90.0, roll=0.0)
        )
        
        parked_car1_spawn = carla.Transform(
            carla.Location(x=195.0, y=5.0, z=0.3),
            carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
        )
        
        parked_car2_spawn = carla.Transform(
            carla.Location(x=202.0, y=5.0, z=0.3),
            carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
        )
        
        # Spawn ego vehicle
        ego_bp = blueprint_library.find('vehicle.audi.tt')
        ego_vehicle = world.spawn_actor(ego_bp, ego_spawn)
        
        # Spawn pedestrian
        pedestrian_bp = blueprint_library.find('walker.pedestrian.0001')
        pedestrian = world.spawn_actor(pedestrian_bp, pedestrian_spawn)
        
        # Spawn parked vehicles
        parked_bp = blueprint_library.find('vehicle.tesla.model3')
        parked_car1 = world.spawn_actor(parked_bp, parked_car1_spawn)
        parked_car2 = world.spawn_actor(parked_bp, parked_car2_spawn)
        
        return ego_vehicle, pedestrian, [parked_car1, parked_car2]
        
    except Exception as e:
        print(f"ERREUR de création des acteurs: {e}")
        raise

def control_pedestrian_movement(pedestrian, simulation_time):
    if simulation_time > 3.0:
        current_location = pedestrian.get_location()
        target_y = 14
        
        if current_location.y < target_y:
            new_location = carla.Location(
                x=current_location.x,
                y=min(current_location.y + 0.08, target_y),
                z=current_location.z
            )
            pedestrian.set_location(new_location)
            return True
    return False

class RealTimePlotter:
    def __init__(self, max_points=200):
        self.max_points = max_points
        self.times = deque(maxlen=max_points)
        self.distances = deque(maxlen=max_points)
        self.ttc_values = deque(maxlen=max_points)
        self.ego_speeds = deque(maxlen=max_points)
        self.pedestrian_speeds = deque(maxlen=max_points)
        
        plt.switch_backend('TkAgg')
        plt.ion()
        
        try:
            self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(10, 8))
            self.fig.suptitle('Analyse AEB en Temps Réel')
            
            self.ax1.set_ylabel('Distance (m)')
            self.ax1.set_title('Distance Ego-Piéton')
            self.ax1.grid(True)
            
            self.ax2.set_ylabel('TTC (s)')
            self.ax2.set_title('Time-To-Collision')
            self.ax2.grid(True)
            self.ax2.set_ylim(0, 10)
            
            self.ax3.set_ylabel('Vitesse (m/s)')
            self.ax3.set_xlabel('Temps (s)')
            self.ax3.set_title('Vitesses des Véhicules/Piéton')
            self.ax3.grid(True)
            
            plt.tight_layout()
            plt.show(block=False)
            self.plotting_enabled = True
        except Exception as e:
            print(f"[WARNING] Impossible d'initialiser matplotlib: {e}")
            self.plotting_enabled = False
    
    def update(self, time_val, distance, ttc, ego_speed, pedestrian_speed):
        if not self.plotting_enabled:
            return
            
        try:
            self.times.append(time_val)
            self.distances.append(distance)
            self.ttc_values.append(min(ttc, 10))
            self.ego_speeds.append(ego_speed)
            self.pedestrian_speeds.append(pedestrian_speed)
            
            if len(self.times) % 5 == 0:
                self._update_plots()
        except Exception as e:
            print(f"[WARNING] Erreur matplotlib: {e}")
            self.plotting_enabled = False
    
    def _update_plots(self):
        try:
            self.ax1.clear()
            self.ax1.plot(list(self.times), list(self.distances), 'b-', linewidth=2, label='Distance')
            self.ax1.axhline(y=2.5, color='r', linestyle='--', label='Seuil collision')
            self.ax1.set_ylabel('Distance (m)')
            self.ax1.set_title('Distance Ego-Piéton')
            self.ax1.legend()
            self.ax1.grid(True)
            
            self.ax2.clear()
            self.ax2.plot(list(self.times), list(self.ttc_values), 'r-', linewidth=2, label='TTC')
            self.ax2.axhline(y=1.5, color='orange', linestyle='--', label='TTC critique')
            self.ax2.set_ylabel('TTC (s)')
            self.ax2.set_title('Time-To-Collision')
            self.ax2.set_ylim(0, 10)
            self.ax2.legend()
            self.ax2.grid(True)
            
            self.ax3.clear()
            self.ax3.plot(list(self.times), list(self.ego_speeds), 'g-', linewidth=2, label='Ego')
            self.ax3.plot(list(self.times), list(self.pedestrian_speeds), 'm-', linewidth=2, label='Piéton')
            self.ax3.set_ylabel('Vitesse (m/s)')
            self.ax3.set_xlabel('Temps (s)')
            self.ax3.set_title('Vitesses des Véhicules/Piéton')
            self.ax3.legend()
            self.ax3.grid(True)
            
            plt.draw()
            plt.pause(0.001)
        except Exception as e:
            print(f"[WARNING] Erreur mise à jour graphique: {e}")
            self.plotting_enabled = False

def main():
    try:
        plotter = RealTimePlotter()
        sim_time = 0.0
        os.environ['SDL_VIDEO_WINDOW_POS'] = '100,100'
        pygame.init()
        screen = pygame.display.set_mode((350, 400))
        pygame.display.set_caption("AEB Pedestrian Test Monitor")
        font = pygame.font.Font(None, 22)
        clock = pygame.time.Clock()
        
        client, world = initialize_carla()
        ego_vehicle, pedestrian, parked_cars = spawn_actors(world)
        tcp_connection = setup_tcp_server(9001)
        
        collision_occurred = False
        aeb_status = "INACTIVE"
        fcw_active = False
        last_ttc = float('inf')
        ego_car_stop = False
        deceleration = 0.0
        obstacle_type = "NONE"
        pedestrian_visible = False
        simulation_time = 0.0
        
        print("\n=== SCENARIO: Pedestrian emerging from behind parked vehicles ===")
        print("Ego vehicle will approach parked cars.")
        print("AEB system should detect pedestrian and respond appropriately.\n")

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            world.tick()
            simulation_time += 0.05
            
            pedestrian_visible = control_pedestrian_movement(pedestrian, simulation_time)
            
            ego_speed = get_speed(ego_vehicle)
            pedestrian_speed = get_speed(pedestrian) if pedestrian_visible else 0.0
            
            distance = ego_vehicle.get_location().distance(pedestrian.get_location())
            
            if pedestrian_visible and distance < 15.0:
                obstacle_type = "PEDESTRIAN"
                target_actor = pedestrian
                target_speed = pedestrian_speed
            else:
                obstacle_type = "VEHICLE"
                target_actor = min(parked_cars, 
                                 key=lambda car: ego_vehicle.get_location().distance(car.get_location()))
                target_speed = 0.0
                distance = ego_vehicle.get_location().distance(target_actor.get_location())
            
            if pedestrian_visible:
                draw_bounding_box(world, pedestrian, Color(r=255, g=165, b=0))
            for parked_car in parked_cars:
                draw_bounding_box(world, parked_car, Color(r=128, g=128, b=128))
            
            relative_speed = ego_speed - target_speed
            ttc = distance / relative_speed if relative_speed > 0 else float('inf')
            last_ttc = ttc if ttc != float('inf') else last_ttc

            send_data(tcp_connection, {
                'MIO_Distance': distance,
                'MIO_Velocity': target_speed,
                'Ego_Velocity': ego_speed
            })

            sim_data = receive_data(tcp_connection)
            if sim_data:
                ego_car_stop = sim_data['egoCarStop']
                fcw_active = sim_data['FCW_Activate']
                deceleration = sim_data['Deceleration']
                aeb_status = "ACTIVE" if sim_data['AEB_Status'] else "INACTIVE"

            control = carla.VehicleControl()

            if ego_car_stop or collision_occurred:
                control.throttle = 0.0
                control.brake = 1.0
                aeb_status = "ACTIVE"
            else:
                control.throttle = max(1.0, 0.4 - deceleration)
                control.brake = min(1.0, deceleration)

            ego_vehicle.apply_control(control)

            if distance <= 2.5 and not collision_occurred:
                print(f"[COLLISION] Distance: {distance:.1f}m | TTC: {ttc:.2f}s | Obstacle: {obstacle_type}")
                collision_occurred = True

            plotter.update(sim_time, distance, ttc, ego_speed, pedestrian_speed)
            
            draw_hud(screen, font, ego_speed * 3.6, control.throttle, control.brake,
                     last_ttc, collision_occurred, aeb_status, fcw_active, 
                     obstacle_type, pedestrian_visible)

            spectator = world.get_spectator()
            ego_transform = ego_vehicle.get_transform()
            spectator.set_transform(carla.Transform(
                ego_transform.location + carla.Location(z=30, x=-15),
                carla.Rotation(pitch=-45)))

            clock.tick(20)
            sim_time += 0.05

    except KeyboardInterrupt:
        print("\nArrêt demandé par l'utilisateur")
    except Exception as e:
        print(f"ERREUR: {e}")
    finally:
        print("Nettoyage des ressources...")
        if 'tcp_connection' in locals():
            tcp_connection.close()
        if 'world' in locals():
            print("Destruction des acteurs...")
            for actor in world.get_actors().filter('*vehicle*'):
                actor.destroy()
            for actor in world.get_actors().filter('*walker*'):
                actor.destroy()
        pygame.quit()
        print("Nettoyage terminé")

if __name__ == '__main__':
    print("Démarrage du test AEB - Piéton derrière véhicules stationnés...")
    main()