import os
import time

from modules.agent import Agent
from modules.utils import HAS_PYGAME, pygame


class _HeadlessClock:
    def __init__(self):
        self._last_tick_s = None

    def tick(self, tick_rate_hz):
        tick_rate_hz = float(tick_rate_hz)
        if tick_rate_hz <= 0.0:
            return

        target_dt_s = 1.0 / tick_rate_hz
        now_s = time.perf_counter()
        if self._last_tick_s is not None:
            elapsed_s = now_s - self._last_tick_s
            if elapsed_s < target_dt_s:
                time.sleep(target_dt_s - elapsed_s)
                now_s = time.perf_counter()
        self._last_tick_s = now_s


class BTRunner:
    def __init__(self, config):
        self.config = config
        self.bt_viz_cfg = config["bt_runner"].get("bt_visualiser", {})
        self.bt_tick_rate = config["bt_runner"]["bt_tick_rate"]

        requested_visualiser = bool(self.bt_viz_cfg.get("enabled", False))
        self.visualiser_enabled = requested_visualiser and HAS_PYGAME
        self.headless_mode = not HAS_PYGAME

        self.screen = None
        self.bt_visualiser = None
        self.background_color = (224, 224, 224)

        if HAS_PYGAME:
            pygame.init()
        elif requested_visualiser:
            print(
                "[BTRunner] pygame is not installed. Starting in headless mode with BT visualiser disabled."
            )

        if self.visualiser_enabled:
            os.environ["SDL_VIDEO_WINDOW_POS"] = "0,30"
            self.screen_height = self.bt_viz_cfg.get("screen_height", 500)
            self.screen_width = self.bt_viz_cfg.get("screen_width", 500)
            self.screen = pygame.display.set_mode(
                (self.screen_width, self.screen_height), pygame.RESIZABLE
            )
            from .bt_visualiser import BTViewer

            self.bt_visualiser = BTViewer(
                direction=self.bt_viz_cfg.get("direction", "Vertical")
            )

        self.clock = pygame.time.Clock() if HAS_PYGAME else _HeadlessClock()
        self.reset()

    def reset(self):
        self.running = True
        self.paused = False
        self.agent = None

        agent_namespace = self.config["agent"].get("namespaces", [])
        self.agent = Agent(agent_namespace)

        scenario_path = self.config["scenario"].get("environment").replace(".", "/")
        behavior_tree_xml = (
            f"{os.path.dirname(os.path.dirname(os.path.abspath(__file__)))}/"
            f"{scenario_path}/{self.config['agent']['behavior_tree_xml']}"
        )
        self.agent.create_behavior_tree(str(behavior_tree_xml))

    async def step(self):
        await self.agent.run_tree()
        self.clock.tick(self.bt_tick_rate)

    def close(self):
        if self.agent:
            self.agent.close()
        if HAS_PYGAME:
            try:
                pygame.quit()
            except Exception:
                pass

    def render(self):
        if not self.visualiser_enabled:
            return

        self.bt_visualiser.render_tree(self.screen, self.agent.tree)

        if self.paused:
            font = pygame.font.Font(None, 48)
            text = font.render("Paused", True, (255, 0, 0))
            self.screen.blit(text, (10, 10))

        pygame.display.flip()

    def handle_keyboard_events(self):
        if not HAS_PYGAME:
            return

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                    self.running = False
                elif event.key == pygame.K_p:
                    self.paused = not self.paused
