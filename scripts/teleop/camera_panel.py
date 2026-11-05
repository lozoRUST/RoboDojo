"""Three live camera viewports in one top-docked Isaac Sim panel."""
import asyncio


class CameraPanel:
    def __init__(self, camera_manager):
        import omni.ui as ui
        import omni.kit.app
        from omni.kit.widget.viewport import ViewportWidget
        from omni.kit.viewport.utility import get_active_viewport_window

        self.widgets = []
        self.window = ui.Window('SO101 Cameras', width=1100, height=300)
        titles = {'cam_head': 'External - Front', 'cam_side': 'External - Side', 'cam_wrist': 'Wrist'}
        with self.window.frame:
            with ui.HStack(spacing=4):
                for name, camera in zip(camera_manager.camera_names[0], camera_manager.cameras[0]):
                    with ui.VStack():
                        ui.Label(titles.get(name, name), height=22, alignment=ui.Alignment.CENTER)
                        widget = ViewportWidget(camera_path=camera.prim_path, resolution=(640, 480))
                        self.widgets.append(widget)
        active = get_active_viewport_window()
        target_name = active.title if active else 'Viewport'

        async def dock():
            for _ in range(3):
                await omni.kit.app.get_app().next_update_async()
            target = ui.Workspace.get_window(target_name)
            if target and self.window:
                self.window.dock_in(target, ui.DockPosition.TOP, 0.38)
                print('[cameras] Three live views docked above main viewport.', flush=True)

        self.dock_task = asyncio.ensure_future(dock())

    def close(self):
        self.dock_task.cancel()
        for widget in self.widgets:
            widget.destroy()
        self.widgets.clear()
        self.window.destroy()
        self.window = None
