import pygame
import sys


class OnScreenDisplay:

    def __init__(self, welcome_message='', fullscreen=False):
        self.pg = pygame
        self.pg.init()
        self.display_dimensions = self._get_display_dimensions()
        self.fullscreen = fullscreen
        self.font = pygame.font.SysFont("ubuntu", 27)
        self.color_text = (0, 185, 0)
        self.color_background = (255, 255, 255)
        self.current_text = welcome_message
        self._initialize_screen(welcome_message)

    def _get_display_dimensions(self):
        return pygame.display.Info().current_w, pygame.display.Info().current_h

    def _set_display_mode(self):
        if self.fullscreen is True:
            self.screen = pygame.display.set_mode(self.display_dimensions, pygame.FULLSCREEN)
        else:
            self.screen = pygame.display.set_mode((640, 480))
        self.screen_rect = self.screen.get_rect()

    def _initialize_screen(self, welcome_message):
        self._set_display_mode()
        self.change_text(welcome_message)

        clock = pygame.time.Clock()  # for limiting FPS
        while True:
            for event in self.pg.event.get():
                if event.type == self.pg.KEYDOWN and event.key == self.pg.K_f:
                    self.fullscreen = not self.fullscreen
                    self._set_display_mode()
                    self.change_text(self.current_text)
            clock.tick(60)

    def change_text(self, new_text):
        text = self.font.render(new_text, True, self.color_text)
        self.screen.fill(self.color_background)
        self.screen.blit(text, text.get_rect(center=self.screen_rect.center))
        self.pg.display.flip()
