import pygame

class App:
    class Player:
        def __init__(self):
            self.x:float = 0
            self.y:float = 0
            self.vel_x:float = 0
            self.vel_y:float = 0

            self.cam_x:float = 0
            self.cam_y:float = 0

            # STATS
            self.speed:float = 10
            self.width:int = 50
            self.height:int = 90
        
        def Run(self):
            self.x += self.vel_x
            self.y += self.vel_y
        
        def Render(self, win:pygame.Surface):
            pygame.draw.rect(win, (255, 0, 0), (self.x-self.cam_x -self.width/2 +win.get_width()/2,
                                                -self.y+self.cam_y -self.height/2 +win.get_height()/2,
                                                self.width, self.height))
        
        def Controls(self):
            keys = pygame.key.get_pressed()
            if keys[pygame.K_w]:
                self.vel_y += self.speed

    def __init__(self) -> None:
        self.win = pygame.display.set_mode((900, 600))
        self.player = self.Player()

    def Run(self):
        self.running = True
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()

            #Logihh
            self.player.Run()

            #Drawin
            self.win.fill((255, 255, 255))
            
            #CAT
            self.player.Render(self.win)

            pygame.display.flip()

App().Run()