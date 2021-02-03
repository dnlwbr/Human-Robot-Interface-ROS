import pygame


class OnScreenDisplay:

    def __init__(self, welcome_message='', fullscreen=True):
        self.pg = pygame
        self.pg.init()
        self.fullscreen = fullscreen
        self.font = pygame.font.SysFont("ubuntu", 27)
        self.color_text = (0, 185, 0)
        self.color_background = (255, 255, 255)
        self._initialize_screen(welcome_message)

    def _initialize_screen(self, welcome_message):
        if self.fullscreen is True:
            self.screen = pygame.display.set_mode(self._get_display_dimensions(), pygame.FULLSCREEN)
        else:
            self.screen = pygame.display.set_mode((640, 480))
        self.screen_rect = self.screen.get_rect()
        self.change_text(welcome_message)

    def _get_display_dimensions(self):
        return pygame.display.Info().current_w, pygame.display.Info().current_h

    def change_text(self, new_text):
        text = self.font.render(new_text, True, self.color_text)
        self.screen.fill(self.color_background)
        self.screen.blit(text, text.get_rect(center=self.screen_rect.center))
        # self.screen.blit(text, (320 - text.get_width() // 2, 240 - text.get_height() // 2))
        self.pg.display.flip()
