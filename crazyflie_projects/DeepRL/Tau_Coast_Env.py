import gym
from gym import logger,spaces
from stable_baselines3.common.env_checker import check_env
import math

import numpy as np


class Tau_Coast_Env():
    """Custom Environment that follows gym interface"""
    metadata = {'render.modes': ['human']}

    def __init__(self):
        super(Tau_Coast_Env, self).__init__()

        self.gravity = 9.8
        self.masscart = 1.0
        self.force_mag = 10.0
        self.dt = 0.02  # seconds between state updates
        self.x_d = 1.0
        self.t_step = 0
        self.Once_flag = False

        # Angle at which to fail the episode
        self.x_threshold = 2.4
        self.t_threshold = 500

        # Angle limit set to 2 * theta_threshold_radians so failing observation
        # is still within bounds.
        high = np.array(
            [
                self.x_threshold * 2,
                np.finfo(np.float32).max,
            ],
            dtype=np.float32,
        )

        self.action_space = spaces.Discrete(3)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

        self.screen_width = 600
        self.screen_height = 400
        self.screen = None
        self.clock = None
        self.isopen = True
        self.state = None

        self.steps_beyond_done = None

    def step(self, action):
        err_msg = f"{action!r} ({type(action)}) invalid"
        assert self.action_space.contains(action), err_msg
        assert self.state is not None, "Call reset before using step method."
        self.t_step += 1
        x,x_dot = self.state


        ## IF ACTION TRIGGERED THEN KEEP THESE DYNAMICS
        if action == 1 or self.Once_flag == True:
            C_drag = 1.0
            self.Once_flag = True
            x_acc = (-C_drag*x_dot)/self.masscart
        else:
            x_acc = 0
        
        x = x + self.dt*x_dot
        x_dot = x_dot + self.dt*x_acc

        self.state = (x, x_dot)


        done = bool(
            x > self.x_threshold
            or self.t_step >= self.t_threshold
        )

        if not done:
            reward = np.clip(1/np.abs(self.x_d-x)**2,0,10)
        elif self.steps_beyond_done is None:
            self.steps_beyond_done = 0
            reward = np.clip(1/np.abs(self.x_d-x)**2,0,10)
        else:
            if self.steps_beyond_done == 0:
                logger.warn(
                    "You are calling 'step()' even though this "
                    "environment has already returned done = True. You "
                    "should always call 'reset()' once you receive 'done = "
                    "True' -- any further steps are undefined behavior."
                )
            self.steps_beyond_done += 1
            reward = 0.0


        return np.array(self.state,dtype=np.float32), reward, done, {}

    def reset(self):
        self.t_step = 0
        self.Once_flag = False
        self.state = np.array([-1.0,1.0])

        return np.array(self.state,dtype=np.float32)

    def render(self):
        import pygame
        from pygame import gfxdraw

        ## DEFINE COLORS
        white = (255,255,255)
        black = (0,0,0)
        blue = (29,123,243)
        red = (255,0,0)

        ## INITIATE SCREEN AND CLOCK ON FIRST LOADING
        if self.screen is None:
            pygame.init()
            self.screen = pygame.display.set_mode((self.screen_width,self.screen_height))
        
        if self.clock is None:
            self.clock = pygame.time.Clock()

        if self.state is None:
            return None
        x = self.state

        ## CREATE BACKGROUND SURFACE
        self.surf = pygame.Surface((self.screen_width, self.screen_height))
        self.surf.fill(white)

        ## CREATE TIMESTEP LABEL
        my_font = pygame.font.SysFont(None, 30)
        text_surface = my_font.render(f'Time Step: {self.t_step:03d}', True, black)

        world_width = self.x_threshold * 2
        scale = self.screen_width / world_width

        

        ## CREATE CART
        cartwidth = 50.0
        cartheight = 30.0

        l, r, t, b = -cartwidth / 2, cartwidth / 2, cartheight / 2, -cartheight / 2
        cartx = x[0] * scale + self.screen_width / 2.0  # MIDDLE OF CART
        carty = 100  # TOP OF CART

        gfxdraw.hline(self.surf, 0, self.screen_width, carty,black)
        gfxdraw.vline(self.surf, int(self.x_d*scale + self.screen_width / 2.0), 0, self.screen_height,black)

        ## CREATE CART COORDS AND TRANSLATE TO PROPER LOCATION
        cart_coords = [(l, b), (l, t), (r, t), (r, b)]
        cart_coords = [(c[0] + cartx, c[1] + carty) for c in cart_coords]
        gfxdraw.filled_polygon(self.surf, cart_coords, black)

        if self.Once_flag == True:
            gfxdraw.aacircle(self.surf,int(cartx),int(carty),5,red)
            gfxdraw.filled_circle(self.surf,int(cartx),int(carty),5,red)
        else:
            gfxdraw.aacircle(self.surf,int(cartx),int(carty),5,blue)
            gfxdraw.filled_circle(self.surf,int(cartx),int(carty),5,blue)



        
        

        ## FLIP IMAGE SO X->RIGHT AND Y->UP
        self.surf = pygame.transform.flip(self.surf, False, True)
        self.screen.blit(self.surf, (0, 0))
        self.screen.blit(text_surface, (5,5))



        self.clock.tick(45)
        pygame.display.flip()

        
    def close(self):
        if self.screen is not None:
            import pygame

            pygame.display.quit()
            pygame.quit()
            self.isopen = False


if __name__ == '__main__':

    env = Tau_Coast_Env()
    for _ in range(5):
        env.reset()
        done = False
        while not done:
            env.render()
            obs,reward,done,info = env.step(env.action_space.sample())


    env.close()
