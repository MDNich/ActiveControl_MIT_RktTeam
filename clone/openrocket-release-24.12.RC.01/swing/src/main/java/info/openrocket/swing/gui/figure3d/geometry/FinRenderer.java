package info.openrocket.swing.gui.figure3d.geometry;

import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.fixedfunc.GLLightingFunc;
import com.jogamp.opengl.fixedfunc.GLMatrixFunc;
import com.jogamp.opengl.glu.GLU;
import com.jogamp.opengl.glu.GLUtessellator;
import com.jogamp.opengl.glu.GLUtessellatorCallback;
import com.jogamp.opengl.glu.GLUtessellatorCallbackAdapter;

import info.openrocket.core.rocketcomponent.EllipticalFinSet;
import info.openrocket.core.rocketcomponent.FinSet;
import info.openrocket.core.rocketcomponent.InsideColorComponent;
import info.openrocket.core.rocketcomponent.TabControlledTrapezoidFinSet;
import info.openrocket.core.util.Transformation;
import info.openrocket.swing.gui.rocketfigure.FinProfileGeometry;
import info.openrocket.swing.gui.rocketfigure.FinSetShapes;
import info.openrocket.core.util.BoundingBox;
import info.openrocket.core.util.Coordinate;
import info.openrocket.swing.gui.figure3d.geometry.Geometry.Surface;

public class FinRenderer {
	// Each render call owns its tessellator; never reuse a deleted native object.
	
	public void renderFinSet(final GL2 gl, FinSet finSet, Surface which) {
		
        GLUtessellator tess = GLU.gluNewTess();
        boolean profiled = finSet.getCrossSection() != FinSet.CrossSection.SQUARE
                || finSet instanceof TabControlledTrapezoidFinSet;
	    BoundingBox bounds = finSet.getInstanceBoundingBox();
		gl.glMatrixMode(GL.GL_TEXTURE);
		gl.glPushMatrix();
		// Mirror the right side fin texture to avoid e.g. mirrored decal text
		if (which == Surface.INSIDE && ((InsideColorComponent) finSet).getInsideColorComponentHandler().isSeparateInsideOutside()) {
			gl.glScaled(-1 / (bounds.max.x - bounds.min.x), 1 / (bounds.max.y - bounds.min.y), 0);
		}
		else {
			gl.glScaled(1 / (bounds.max.x - bounds.min.x), 1 / (bounds.max.y - bounds.min.y), 0);
		}
		gl.glTranslated(-bounds.min.x, -bounds.min.y - finSet.getBodyRadius(), 0);
		gl.glMatrixMode(GLMatrixFunc.GL_MODELVIEW);

		Coordinate[] finPoints = finSet.getFinPointsWithLowResRoot();
		Coordinate[] tabPoints = finSet.getTabPointsWithRootLowRes();

		{
		    gl.glPushMatrix();

            gl.glTranslated(0, - finSet.getBodyRadius(), 0);		// Move to the parent centerline
            
            gl.glRotated( Math.toDegrees(finSet.getCantAngle()), 0, 1, 0);

            if (profiled) {
                var profile = new FinProfileGeometry(finSet);
                renderProfile(gl, profile.faces(FinSetShapes.previewFinOutline(finSet)), finSet.getBodyRadius(), which, Transformation.IDENTITY);
                if (finSet instanceof TabControlledTrapezoidFinSet controlled)
                    renderProfile(gl, profile.faces(controlled.getRollCtrlTabNeutralPoints()), finSet.getBodyRadius(), which,
                            controlled.getRollCtrlTabRotation());
            }

            GLUtessellatorCallback cb = new GLUtessellatorCallbackAdapter() {
				@Override
				public void vertex(Object vertexData) {
					double[] d = (double[]) vertexData;
					gl.glTexCoord2d(d[0], d[1]);
					gl.glVertex3dv(d, 0);
				}
				
				@Override
				public void begin(int type) {
					gl.glBegin(type);
				}
				
				@Override
				public void end() {
					gl.glEnd();
				}

				@Override
				public void combine(double[] coords, Object[] data, float[] weight, Object[] outData) {
					double[] vertex = new double[3];
					vertex[0] = coords[0];
					vertex[1] = coords[1];
					vertex[2] = coords[2];
					outData[0] = vertex;
				}
			};
			
			GLU.gluTessCallback(tess, GLU.GLU_TESS_VERTEX, cb);
			GLU.gluTessCallback(tess, GLU.GLU_TESS_BEGIN, cb);
			GLU.gluTessCallback(tess, GLU.GLU_TESS_END, cb);
			GLU.gluTessCallback(tess, GLU.GLU_TESS_COMBINE, cb);

			// fin side: +z
			if (!profiled && finSet.getSpan() > 0 && which == Surface.INSIDE) {		// Right side
				GLU.gluTessBeginPolygon(tess, null);
				GLU.gluTessBeginContour(tess);
				gl.glNormal3f(0, 0, 1);
				for (int i = finPoints.length - 1; i >= 0; i--) {
					Coordinate c = finPoints[i];
					double[] p = new double[]{c.x, c.y + finSet.getBodyRadius(),
							c.z + finSet.getThickness() / 2.0};
					GLU.gluTessVertex(tess, p, 0, p);
				}
				GLU.gluTessEndContour(tess);
				GLU.gluTessEndPolygon(tess);
			}
			// tab side: +z
			if (finSet.getTabHeight() > 0 && finSet.getTabLength() > 0 && which == Surface.INSIDE) {		// Right side
				GLU.gluTessBeginPolygon(tess, null);
				GLU.gluTessBeginContour(tess);
				gl.glNormal3f(0, 0, 1);
                for (Coordinate c : tabPoints) {
                    double[] p = new double[]{c.x, c.y + finSet.getBodyRadius(),
                            c.z + finSet.getThickness() / 2.0};
                    GLU.gluTessVertex(tess, p, 0, p);
                }
				GLU.gluTessEndContour(tess);
				GLU.gluTessEndPolygon(tess);
			}
			
			// fin side: -z
			if (!profiled && finSet.getSpan() > 0 && which == Surface.OUTSIDE) {		// Left side
				GLU.gluTessBeginPolygon(tess, null);
				GLU.gluTessBeginContour(tess);
				gl.glNormal3f(0, 0, -1);
				for (Coordinate c : finPoints) {
					double[] p = new double[]{c.x, c.y + finSet.getBodyRadius(),
							c.z - finSet.getThickness() / 2.0};
					GLU.gluTessVertex(tess, p, 0, p);

				}
				GLU.gluTessEndContour(tess);
				GLU.gluTessEndPolygon(tess);
			}
			// tab side: -z
			if (finSet.getTabHeight() > 0 && finSet.getTabLength() > 0 && which == Surface.OUTSIDE) {		// Left side
				GLU.gluTessBeginPolygon(tess, null);
				GLU.gluTessBeginContour(tess);
				gl.glNormal3f(0, 0, -1);
				for (int i = tabPoints.length - 1; i >= 0; i--) {
					Coordinate c = tabPoints[i];
					double[] p = new double[]{c.x, c.y + finSet.getBodyRadius(),
							c.z - finSet.getThickness() / 2.0};
					GLU.gluTessVertex(tess, p, 0, p);

				}
				GLU.gluTessEndContour(tess);
				GLU.gluTessEndPolygon(tess);
			}

			// delete tessellator after processing
			GLU.gluDeleteTess(tess);
			
			// Fin strip around the edge
			if (!profiled && finSet.getSpan() > 0 && which == Surface.EDGES) {
				if (!(finSet instanceof EllipticalFinSet))
					gl.glShadeModel(GLLightingFunc.GL_FLAT);
				gl.glBegin(GL.GL_TRIANGLE_STRIP);
				for (int i = 0; i <= finPoints.length; i++) {
					Coordinate c = finPoints[i % finPoints.length];
					// if ( i > 1 ){
					Coordinate c2 = finPoints[(i - 1 + finPoints.length)
							% finPoints.length];
					gl.glNormal3d(c2.y - c.y, c.x - c2.x, 0);
					// }
					gl.glTexCoord2d(c.x, c.y + finSet.getBodyRadius());
					gl.glVertex3d(c.x, c.y + finSet.getBodyRadius(),
							c.z - finSet.getThickness() / 2.0);
					gl.glVertex3d(c.x, c.y + finSet.getBodyRadius(),
							c.z + finSet.getThickness() / 2.0);
				}
				gl.glEnd();
			}
			// Tab strip around the edge
			if (finSet.getTabHeight() > 0 && finSet.getTabLength() > 0 && which == Surface.EDGES) {
				if (!(finSet instanceof EllipticalFinSet))
					gl.glShadeModel(GLLightingFunc.GL_FLAT);
				gl.glBegin(GL.GL_TRIANGLE_STRIP);
				for (int i = tabPoints.length; i >= 0; i--) {
					Coordinate c = tabPoints[i % tabPoints.length];
					// if ( i > 1 ){
					Coordinate c2 = tabPoints[(i - 1 + tabPoints.length)
							% tabPoints.length];
					gl.glNormal3d(c2.y - c.y, c.x - c2.x, 0);
					// }
					gl.glTexCoord2d(c.x, c.y + finSet.getBodyRadius());
					gl.glVertex3d(c.x, c.y + finSet.getBodyRadius(),
							c.z - finSet.getThickness() / 2.0);
					gl.glVertex3d(c.x, c.y + finSet.getBodyRadius(),
							c.z + finSet.getThickness() / 2.0);
				}
				gl.glEnd();
			}
			if (!(finSet instanceof EllipticalFinSet))
				gl.glShadeModel(GLLightingFunc.GL_SMOOTH);
			
			gl.glPopMatrix();
		}
		
		gl.glMatrixMode(GL.GL_TEXTURE);
		gl.glPopMatrix();
		gl.glMatrixMode(GLMatrixFunc.GL_MODELVIEW);
		
	}
    private static void renderProfile(GL2 gl, java.util.List<FinProfileGeometry.Face> faces,
            double bodyRadius, Surface which, Transformation transform) {
        int side = which == Surface.INSIDE ? 1 : which == Surface.OUTSIDE ? -1 : 0;
        gl.glBegin(GL2.GL_QUADS);
        for (var face : faces) {
            if (face.side() != side) continue;
            Coordinate[] p = face.points();
            Coordinate normal = p[1].sub(p[0]).cross(p[3].sub(p[0]));
            if (normal.length() < 1e-18) normal = p[2].sub(p[1]).cross(p[0].sub(p[1]));
            if (normal.length() < 1e-18) continue;
            // Small but valid fin facets fall below Coordinate.normalize()'s general tolerance.
            normal = transform.linearTransform(normal.multiply(1 / normal.length()));
            gl.glNormal3d(normal.x, normal.y, normal.z);
            for (Coordinate vertex : p) {
                gl.glTexCoord2d(vertex.x, vertex.y + bodyRadius);
                Coordinate rotated = transform.transform(vertex);
                gl.glVertex3d(rotated.x, rotated.y + bodyRadius, rotated.z);
            }
        }
        gl.glEnd();
    }

}
