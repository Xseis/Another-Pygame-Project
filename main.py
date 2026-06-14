import pygame
import math
pygame.init()

class ObjectClass:
    def __init__(self, x:float, y:float, length:float=1, width:float=1) -> None:
        self.x = x
        self.y = y
        
        self.length = length
        self.width = width
    
    def get_corners(self) -> list[tuple[float, float]]:
        """Returns the 4 corners of a non-rotated rectangle"""
        hl, hw = self.length/2, self.width/2
        local_corners = [(-hl, -hw), (-hl, hw), (hl, hw), (hl, -hw)]
        return [(self.x+dx, self.y+dy) for dx, dy in local_corners] 

    def get_aabb(self):
        corners = self.get_corners()
        xs = [corner[0] for corner in corners]
        ys = [corner[1] for corner in corners]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        return (min_x, min_y, max_x-min_x, max_y-min_y)

class PhysicsObjectClass(ObjectClass):
    def __init__(self, x:float, y:float, angle:float, mass:float, inertia:float, grid:"GridClass", length:float=1, width:float=1, is_static:bool=False, can_collide:bool=True) -> None:
        super().__init__(x, y, length, width)
        self.vx = 0.0
        self.vy = 0.0
        self.angle = angle
        self.angular_vel = 0.0

        self.mass = mass
        self.inertia = inertia
        self.is_static = is_static
        self.can_collide = can_collide

        self.drag = 0.4

        # For collision
        if self.can_collide:
            self.grid = grid
            self.chunks = grid.query(self)
            self.grid.add_object(self)
    
    def get_corners(self) -> list[tuple[float, float]]:
        """Returns the 4 corners of a rotated rectangle"""
        hl, hw = self.length/2, self.width/2
        cos_a, sin_a = math.cos(self.angle), math.sin(self.angle)
        local_corners = [(-hl, -hw), (-hl, hw), (hl, hw), (hl, -hw)]
        return [(self.x + dx*cos_a - dy*sin_a, self.y + dx*sin_a + dy*cos_a) for dx, dy in local_corners] 

    def resolve_collision(self):
        if self.is_static or not self.can_collide:
            return
        chunks = self.grid.query(self)
        collision_objects = self.grid.get_objects(chunks) - set([self])
        for obj in collision_objects:
            is_colliding, normal, depth = self.SeparatingAxisTheorem(self, obj)
            if is_colliding and normal and depth:
                mtv = (normal[0]*depth, normal[1]*depth)
                self.x -= mtv[0]
                self.y -= mtv[1]

                impact = self.vx*normal[0] + self.vy*normal[1]
                if impact > 0:
                    self.vx -= normal[0] * impact*1.1
                    self.vy -= normal[1] * impact*1.1
        
    def SeparatingAxisTheorem(self, obj1:"PhysicsObjectClass", obj2:"PhysicsObjectClass"):
        obj1_corners = obj1.get_corners()
        obj2_corners = obj2.get_corners()
        axes = []

        # Get axes
        for corners in (obj1_corners, obj2_corners):
            for i in range(4):
                c1 = corners[i]
                c2 = corners[(i+1) % 4]

                edge_x = c2[0] - c1[0]
                edge_y = c2[1] - c1[1]
                axis_x = -edge_y
                axis_y = edge_x

                length = math.hypot(axis_x, axis_y)
                axis_x /= length
                axis_y /= length

                axes.append((axis_x, axis_y))

        smallest_overlap = float("inf")

        for axis_x, axis_y in axes:
            min_a = max_a = obj1_corners[0][0] * axis_x + obj1_corners[0][1] * axis_y
            for x, y in obj1_corners[1:]:
                proj = x * axis_x + y * axis_y
                min_a = min(min_a, proj)
                max_a = max(max_a, proj)
            min_b = max_b = obj2_corners[0][0] * axis_x + obj2_corners[0][1] * axis_y
            for x, y in obj2_corners[1:]:
                proj = x * axis_x + y * axis_y
                min_b = min(min_b, proj)
                max_b = max(max_b, proj)

            overlap = min(max_a, max_b) - max(min_a, min_b)
            if overlap <= 0:
                return False, None, 0
            if overlap < smallest_overlap:
                smallest_overlap = overlap
                smallest_axis = (axis_x, axis_y)
        
        # Centers
        center_a = (
            sum(x for x, y in obj1_corners) / len(obj1_corners),
            sum(y for x, y in obj1_corners) / len(obj1_corners)
        )
        center_b = (
            sum(x for x, y in obj2_corners) / len(obj2_corners),
            sum(y for x, y in obj2_corners) / len(obj2_corners)
        )
        dir_x = center_b[0] - center_a[0]
        dir_y = center_b[1] - center_a[1]

        if dir_x * smallest_axis[0] + dir_y * smallest_axis[1] < 0:
            smallest_axis = (-smallest_axis[0], -smallest_axis[1])

        return True, smallest_axis, smallest_overlap
        
    def tick(self, dt):
        if self.is_static:
            return
        old_x, old_y = self.x, self.y
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.angle += self.angular_vel * dt

        # Update collision chunks / grid
        if old_x != self.x or old_y != self.y: # If position changed
            self.grid.update_chunks(self)

        # Collision check 
        # SAT (Separating Axis Theorem) also pushes objects away from each other
        # when a collision occurs.
        if self.can_collide:
            self.resolve_collision() 

        # Drag
        self.vx *= self.drag ** dt
        self.vy *= self.drag ** dt
        self.angular_vel *= self.drag ** dt

    @property
    def speed(self):
        return math.hypot(self.vx, self.vy) 

class CatClass(PhysicsObjectClass):
    def __init__(self, x:float, y:float, angle:float, mass:float, inertia:float, grid:"GridClass", length:float=2, width:float=1) -> None:
        super().__init__(x, y, angle, mass, inertia, grid, length, width)

        # Controls
        self.gas = 0   # 0..1
        self.brake = 0 # 0..1
        self.turn = 0  # -1..1
        
        self.max_turn = math.radians(45)
        self.engine_force = 20000
        self.default_drag = 0.2
        self.handbreak = False
        self.handbreak_duration = 0 # Used to slow turn

        # Combos
        self.goodslip = False
        self.goodslip_boost_timer = 0.0
        self.goodslip_boost_duration = 0.5 # sec
        self.goodslip_boost_multiplier = 2
    
    def Controls(self, dt):
        keys = pygame.key.get_pressed()
        self.brake = 0
        self.turn = 0
        self.gas = 0
        if keys[pygame.K_w]:
            self.gas += 1
        if keys[pygame.K_a]:
            self.turn -= 1
        if keys[pygame.K_d]:
            self.turn += 1
        if keys[pygame.K_SPACE]:
            if not self.handbreak:
                self.angular_vel += self.turn*5
            self.gas = 1 * abs(self.turn)
            self.handbreak = True
            self.handbreak_duration += 1 * dt
        else:
            self.handbreak = False
            self.handbreak_duration = 0
        if self.handbreak: 
            if self.turn != 0:
                self.drag = 0.3
                self.gas *= 0.8
        else: self.drag = self.default_drag

    def Update(self, dt):
        self.Controls(dt)
        self.vx += (self.gas * self.engine_force / self.mass) * math.cos(self.angle) * dt
        self.vy += (self.gas * self.engine_force / self.mass) * math.sin(self.angle) * dt

        if self.goodslip_boost_timer > 0:
            boost_strength = (self.goodslip_boost_timer / self.goodslip_boost_duration)
            boost_force = self.engine_force * self.goodslip_boost_multiplier * boost_strength
            self.vx += (boost_force / self.mass ) * math.cos(self.angle) * dt
            self.vy += (boost_force / self.mass) * math.sin(self.angle) * dt

            self.goodslip_boost_timer -= dt

        # Lateral grip
        forward = (math.cos(self.angle), math.sin(self.angle))
        right   = (-math.sin(self.angle), math.cos(self.angle))

        forward_speed = self.vx * forward[0] + self.vy * forward[1]
        lateral_speed = self.vx * right[0]   + self.vy * right[1]

        grip = 1 if self.handbreak else 0.1
        lateral_speed *= grip ** dt

        self.vx = forward[0] * forward_speed + right[0] * lateral_speed
        self.vy = forward[1] * forward_speed + right[1] * lateral_speed

        # Steering
        steering_angle = self.turn * self.max_turn
        slip = abs(lateral_speed) / (self.speed + 0.001)
        if slip >= 0.6 and not self.goodslip and self.handbreak:
            self.goodslip = True
        if self.goodslip and slip < 0.5 and not self.handbreak:
            self.goodslip = False
            self.goodslip_boost_timer = self.goodslip_boost_duration # Reset timer
        
        target_angular_vel = self.speed * math.tan(steering_angle) / self.length
        target_angular_vel *= (1 - slip * 0.9)
        max_lateral_accel = 3200 if not self.handbreak else 3200/2* max((2-self.handbreak_duration), 0)

        if self.speed > 0:
            max_angular_vel = max_lateral_accel / self.speed
            target_angular_vel = max(-max_angular_vel, min(max_angular_vel, target_angular_vel))

        steer_response = 10.0
        self.angular_vel += (target_angular_vel - self.angular_vel) * (steer_response / self.inertia) * dt

        super().tick(dt)

class CameraClass(PhysicsObjectClass):
    def __init__(self, x:float, y:float, angle:float, game:"GameClass") -> None:
        super().__init__(x, y, angle, mass=0, inertia=0, grid=game.grid, is_static=True, can_collide=False)
        self.target:PhysicsObjectClass | None = None
        self.zoom = 0.5
        self.game = game
    
    def follow(self):
        if not self.target:
            return
        self.x = self.target.x
        self.y = self.target.y
    
    def cam_space(self, x, y):
        return (x-self.x)*self.zoom+self.game.WIDTH/2, (y-self.y)*self.zoom+self.game.HEIGHT/2


class GridClass:
    def __init__(self, grid_size:int=100) -> None:
        self.grid:dict[tuple[int, int], set[PhysicsObjectClass]] = {}
        self.grid_size = grid_size
    
    def query(self, obj:ObjectClass) -> list[tuple[int, int]]:
        aabb = obj.get_aabb()
        start_x = int(aabb[0] // self.grid_size)
        start_y = int(aabb[1] // self.grid_size)
        end_x = int((aabb[0]+aabb[2]) // self.grid_size)
        end_y = int((aabb[1]+aabb[3]) // self.grid_size)

        grids = []
        for x in range(start_x, end_x+1):
            for y in range(start_y, end_y+1):
                grids.append((x, y))
        return grids
    
    def add_object(self, obj:PhysicsObjectClass):
        chunks = self.query(obj)
        for chunk in chunks:
            if self.grid.get(chunk):
                self.grid[chunk].add(obj)
            else: 
                self.grid[chunk] = set([obj])
    
    def get_objects(self, grids:list[tuple[int, int]]) -> set[PhysicsObjectClass]:
        objects = set()
        for chunk in grids:
            if chunk in self.grid:
                objects.update(self.grid[chunk])
        return objects
    
    def update_chunks(self, obj:PhysicsObjectClass):
        new_chunks = self.query(obj)
        new_chunks_set = set(new_chunks)
        old_chunks_set = set(obj.chunks)
        for chunk in old_chunks_set - new_chunks_set:
            self.grid[chunk].discard(obj)
            if not self.grid[chunk]: # Delete the chunk if no objects are inside
                del self.grid[chunk] # because it cleans up memory :3
        # Set object in new grid chunks
        for chunk in new_chunks_set - old_chunks_set:
            self.grid.setdefault(chunk, set()).add(obj)
        # Set new object chunks
        obj.chunks = new_chunks        

class GameClass:
    def __init__(self) -> None:
        self.grid = GridClass(grid_size=500)
        self.cat = CatClass(x=400.0, y=400.0, angle=0.0, mass=10.0, inertia=1.0, grid=self.grid, width=100, length=160)
        self.wall = PhysicsObjectClass(0, 0, 0, 0, 0, grid=self.grid, width=100, length=100)
        self.wall.is_static = True

        self.camera = CameraClass(0, 0, 0, self)
        self.camera.target = self.cat

        self.objects:list[ObjectClass] = [self.cat, self.wall] # PURELY DRAWING/VISUAL
        for i in range(10):
            wall= PhysicsObjectClass(i*200, 0, 0, 0, 0, grid=self.grid, width=100, length=100)
            self.objects.append(wall)

        self.WIDTH, self.HEIGHT = 1000, 600
        self.win = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        self.clock = pygame.time.Clock()
    
    def Run(self):
        self.running = True
        dt = 0

        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                if event.type == pygame.MOUSEWHEEL:
                    self.camera.zoom *= 1.1 if event.y == 1 else 0.9
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_z:
                        self.camera.zoom = 1

            self.cat.Update(dt)
            self.camera.follow()
            
            self.win.fill((255, 255, 255))
            gridw = min(int(self.WIDTH/self.camera.zoom//self.grid.grid_size+3), 100)
            gridh = min(int(self.HEIGHT/self.camera.zoom//self.grid.grid_size+3), 100)
            for x in range(gridw):
                for y in range(gridh):
                    dx, dy = x-gridw//2, y-gridh//2
                    pygame.draw.rect(self.win, (50, 255, 90), (self.WIDTH/2+dx*self.grid.grid_size*self.camera.zoom-(self.camera.x%(self.grid.grid_size)*self.camera.zoom),
                                                        self.HEIGHT/2+dy*self.grid.grid_size*self.camera.zoom-(self.camera.y%(self.grid.grid_size)*self.camera.zoom), 
                                                        self.grid.grid_size*self.camera.zoom+1, 
                                                        self.grid.grid_size*self.camera.zoom+1), 1)
            for obj in self.objects:
                corners = [self.camera.cam_space(dx, dy) for dx, dy in obj.get_corners()]
                pygame.draw.polygon(self.win, (255, 0, 0), corners)
            # CAR AABB
            aabb = self.cat.get_aabb()
            screen_pos = self.camera.cam_space(aabb[0], aabb[1])
            pygame.draw.rect(self.win, (80,0,0), (*screen_pos, aabb[2]*self.camera.zoom, aabb[3]*self.camera.zoom), 1)

            # CAR DIRECTION
            if self.cat.speed > 0:
                angle = math.atan2(self.cat.vy, self.cat.vx)
            else:
                angle = self.cat.angle
            pygame.draw.line(self.win, (80,0,0), self.camera.cam_space(self.cat.x, self.cat.y), self.camera.cam_space(self.cat.x+math.cos(angle)*70, self.cat.y+math.sin(angle)*70))
            pygame.draw.line(self.win, (80,0,0), self.camera.cam_space(self.cat.x, self.cat.y), self.camera.cam_space(self.cat.x+math.cos(self.cat.angle)*100, self.cat.y+math.sin(self.cat.angle)*100))
            
            # SPEEDOMETER
            center = (self.WIDTH-100, self.HEIGHT-100)
            kmh = self.cat.speed/100*3.6
            angle = math.radians(180)
            angle += math.radians(kmh/80*180)
            pygame.draw.line(self.win, (0,0,0), center, 
                            (center[0]+math.cos(angle)*80, center[1]+math.sin(angle)*80),
                            4)
            pygame.draw.arc(self.win, (0,0,0), (center[0]-90, center[1]-90, 180, 180), 0, math.radians(360), 4)
            font = pygame.font.Font(None, 40)
            self.win.blit(font.render(f"{int(kmh)}km/h", True, (0,0,0)), (center[0]-50, center[1]+40))

            # Coords
            font = pygame.font.Font(None, 20)
            self.win.blit(font.render(f"x: {int(self.camera.x/100)}m", True, (0,0,0), (220,220,220)), (10, self.HEIGHT-50))
            self.win.blit(font.render(f"y: {int(self.camera.y/100)}m", True, (0,0,0), (220,220,220)), (10, self.HEIGHT-30))


            pygame.display.flip()
            dt = self.clock.tick() / 1000
            pygame.display.set_caption(f"FPS: {int(self.clock.get_fps())}")
        pygame.quit()

GameClass().Run()