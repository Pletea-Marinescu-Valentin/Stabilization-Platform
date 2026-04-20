"""
HoldMyCoffee Dashboard - 3D Platform Visualization Widget
Uses PyOpenGL + PyQt6 to render a tilting platform in real-time.
"""

import math
import numpy as np
from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor

try:
    from PyQt6.QtOpenGLWidgets import QOpenGLWidget
    from OpenGL.GL import *
    from OpenGL.GLU import *
    HAS_OPENGL = True
except ImportError:
    HAS_OPENGL = False
    QOpenGLWidget = QWidget


class PlatformView3D(QOpenGLWidget):
    """3D visualization of the stabilization platform."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.pitch = 0.0
        self.roll = 0.0
        self.height = 0.0
        self.target_pitch = 0.0
        self.target_roll = 0.0
        self.setMinimumSize(300, 300)

    def set_orientation(self, pitch: float, roll: float, height: float = 0.0):
        self.pitch = pitch
        self.roll = roll
        self.height = height
        self.update()

    def set_targets(self, target_pitch: float, target_roll: float):
        self.target_pitch = target_pitch
        self.target_roll = target_roll
        self.update()

    def initializeGL(self):
        glClearColor(0.12, 0.12, 0.15, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glShadeModel(GL_SMOOTH)

        glLightfv(GL_LIGHT0, GL_POSITION, [5.0, 10.0, 7.0, 1.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.3, 0.3, 0.3, 1.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.8, 0.8, 0.8, 1.0])

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect = w / max(h, 1)
        gluPerspective(35, aspect, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        gluLookAt(4.0, 5.0, 4.0, 0, 0.5, 0, 0, 1, 0)

        # Grid floor
        self._draw_grid()

        # Base cylinder
        glPushMatrix()
        glColor3f(0.3, 0.3, 0.35)
        self._draw_cylinder(0.3, 0.0, 0.6, 20)
        glPopMatrix()

        # Support column
        h_offset = self.height * 0.01  # scale height for visual
        glPushMatrix()
        glColor3f(0.45, 0.45, 0.5)
        self._draw_cylinder(0.12, 0.6, 0.6 + 1.0 + h_offset, 12)
        glPopMatrix()

        # Platform (tilted)
        glPushMatrix()
        glTranslatef(0, 1.6 + h_offset, 0)
        glRotatef(self.pitch, 1, 0, 0)
        glRotatef(self.roll, 0, 0, 1)

        # Platform disc
        glColor3f(0.18, 0.55, 0.85)
        self._draw_disc(1.2, 0.08, 32)

        # Cup (cylinder on platform)
        glPushMatrix()
        glColor3f(0.9, 0.85, 0.7)
        self._draw_cylinder(0.2, 0.08, 0.55, 16)
        # Coffee surface
        glColor3f(0.35, 0.2, 0.1)
        self._draw_circle(0.18, 0.5, 16)
        glPopMatrix()

        glPopMatrix()

        # Target ghost platform (semi-transparent)
        glPushMatrix()
        glTranslatef(0, 1.6, 0)
        glRotatef(self.target_pitch, 1, 0, 0)
        glRotatef(self.target_roll, 0, 0, 1)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glColor4f(0.2, 0.8, 0.3, 0.15)
        self._draw_disc(1.2, 0.02, 32)
        glDisable(GL_BLEND)
        glPopMatrix()

    def _draw_grid(self):
        glDisable(GL_LIGHTING)
        glColor3f(0.22, 0.22, 0.25)
        glBegin(GL_LINES)
        for i in range(-3, 4):
            glVertex3f(i, 0, -3)
            glVertex3f(i, 0, 3)
            glVertex3f(-3, 0, i)
            glVertex3f(3, 0, i)
        glEnd()
        glEnable(GL_LIGHTING)

    def _draw_cylinder(self, radius, y_bottom, y_top, slices):
        for i in range(slices):
            a0 = 2 * math.pi * i / slices
            a1 = 2 * math.pi * (i + 1) / slices
            x0, z0 = radius * math.cos(a0), radius * math.sin(a0)
            x1, z1 = radius * math.cos(a1), radius * math.sin(a1)
            nx0, nz0 = math.cos(a0), math.sin(a0)
            nx1, nz1 = math.cos(a1), math.sin(a1)
            glBegin(GL_QUADS)
            glNormal3f(nx0, 0, nz0)
            glVertex3f(x0, y_bottom, z0)
            glVertex3f(x0, y_top, z0)
            glNormal3f(nx1, 0, nz1)
            glVertex3f(x1, y_top, z1)
            glVertex3f(x1, y_bottom, z1)
            glEnd()

    def _draw_disc(self, radius, thickness, slices):
        # Top face
        glNormal3f(0, 1, 0)
        glBegin(GL_TRIANGLE_FAN)
        glVertex3f(0, thickness, 0)
        for i in range(slices + 1):
            a = 2 * math.pi * i / slices
            glVertex3f(radius * math.cos(a), thickness, radius * math.sin(a))
        glEnd()
        # Bottom face
        glNormal3f(0, -1, 0)
        glBegin(GL_TRIANGLE_FAN)
        glVertex3f(0, 0, 0)
        for i in range(slices + 1):
            a = 2 * math.pi * i / slices
            glVertex3f(radius * math.cos(a), 0, radius * math.sin(a))
        glEnd()
        # Side
        self._draw_cylinder(radius, 0, thickness, slices)

    def _draw_circle(self, radius, y, slices):
        glNormal3f(0, 1, 0)
        glBegin(GL_TRIANGLE_FAN)
        glVertex3f(0, y, 0)
        for i in range(slices + 1):
            a = 2 * math.pi * i / slices
            glVertex3f(radius * math.cos(a), y, radius * math.sin(a))
        glEnd()


class PlatformView3DFallback(QWidget):
    """Fallback 2D view if OpenGL is not available."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.pitch = 0.0
        self.roll = 0.0
        self.height = 0.0
        self.setMinimumSize(300, 300)

    def set_orientation(self, pitch, roll, height=0.0):
        self.pitch = pitch
        self.roll = roll
        self.height = height
        self.update()

    def set_targets(self, target_pitch, target_roll):
        self.update()

    def paintEvent(self, event):
        from PyQt6.QtGui import QPainter, QPen, QBrush, QFont
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.fillRect(self.rect(), QColor(30, 30, 38))

        cx, cy = self.width() // 2, self.height() // 2
        r = min(cx, cy) - 30

        # Platform circle
        p.setPen(QPen(QColor(70, 160, 230), 3))
        p.setBrush(QBrush(QColor(46, 140, 215, 100)))
        p.drawEllipse(cx - r, cy - r, 2 * r, 2 * r)

        # Tilt indicator
        dx = r * 0.8 * math.sin(math.radians(self.roll))
        dy = r * 0.8 * math.sin(math.radians(self.pitch))
        p.setPen(QPen(QColor(255, 100, 80), 4))
        p.drawLine(int(cx - dx), int(cy - dy), int(cx + dx), int(cy + dy))

        # Labels
        p.setPen(QColor(200, 200, 200))
        p.setFont(QFont("Segoe UI", 10))
        p.drawText(10, 20, f"Pitch: {self.pitch:.1f}°")
        p.drawText(10, 40, f"Roll: {self.roll:.1f}°")
        p.end()


def create_platform_view(parent=None):
    """Factory: return OpenGL view if available, else fallback."""
    if HAS_OPENGL:
        return PlatformView3D(parent)
    return PlatformView3DFallback(parent)
