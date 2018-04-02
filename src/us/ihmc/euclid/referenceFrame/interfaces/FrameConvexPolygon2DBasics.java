package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;

/**
 * Write and read interface for a convex polygon defined in the XY-plane and that is expressed in in
 * a changeable reference frame, i.e. the reference frame in which this polygon is expressed can be
 * changed.
 * <p>
 * This implementation of convex polygon is designed for garbage free operations.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameConvexPolygon2DBasics extends FixedFrameConvexPolygon2DBasics, FrameChangeable
{
   /**
    * Clear this polygon and sets it reference frame.
    * <p>
    * After calling this method, the polygon has no vertex, area, or centroid.
    * </p>
    * 
    * @param referenceFrame the new reference frame for this frame convex polygon.
    * @see #clear()
    */
   default void clear(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      clear();
   }

   /**
    * Clear this polygon, sets it reference frame.
    * <p>
    * After calling this method, the polygon has no vertex, area, or centroid. Use only when an empty polygon is desired.
    * </p>
    * 
    * @param referenceFrame the new reference frame for this frame convex polygon.
    * @see #clearAndUpdate()
    */
   default void clearAndUpdate(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      clearAndUpdate();
   }

   /**
    * Sets the reference frame of this polygon without updating or modifying the coordinates of its
    * vertices.
    *
    * @param referenceFrame the new reference frame for this frame convex polygon.
    */
   void setReferenceFrame(ReferenceFrame referenceFrame);

   /**
    * Clears this polygon, adds a single vertex at (0, 0), and updates it.
    * 
    * @param referenceFrame the new reference frame for this frame convex polygon.
    * @see #setToZero()
    */
   default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      FixedFrameConvexPolygon2DBasics.super.setToZero();
   }

   /**
    * Clears this polygon, adds a single vertex at ({@link Double#NaN}, {@link Double#NaN}), and
    * updates it.
    * 
    * @param referenceFrame the new reference frame for this frame convex polygon.
    * @see #setToNaN()
    */
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      FixedFrameConvexPolygon2DBasics.super.setToNaN();
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #setReferenceFrame(ReferenceFrame)}
    * <li>{@link #set(Vertex2DSupplier)}.
    * </ol>
    * 
    * @param referenceFrame the new reference frame for this frame convex polygon.
    * @param vertex2DSupplier the supplier of vertices.
    * @see Vertex2DSupplier
    * @see #set(Vertex2DSupplier)
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Vertex2DSupplier vertex2DSupplier)
   {
      setReferenceFrame(referenceFrame);
      set(vertex2DSupplier);
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #setReferenceFrame(ReferenceFrame)}
    * <li>{@link #set(Vertex3DSupplier)}.
    * </ol>
    * 
    * @param referenceFrame the new reference frame for this frame convex polygon.
    * @param vertex3DSupplier the supplier of vertices.
    * @see Vertex3DSupplier
    * @see #set(Vertex3DSupplier)
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Vertex3DSupplier vertex3DSupplier)
   {
      setReferenceFrame(referenceFrame);
      set(vertex3DSupplier);
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #setReferenceFrame(ReferenceFrame)}
    * <li>{@link #set(Vertex2DSupplier, Vertex2DSupplier)}.
    * </ol>
    * 
    * @param referenceFrame the new reference frame for this frame convex polygon.
    * @param firstVertexSupplier the first supplier of vertices.
    * @param secondVertexSupplier the second supplier of vertices.
    * @see Vertex2DSupplier
    * @see #set(Vertex2DSupplier, Vertex2DSupplier)
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Vertex2DSupplier firstVertexSupplier, Vertex2DSupplier secondVertexSupplier)
   {
      setReferenceFrame(referenceFrame);
      set(firstVertexSupplier, secondVertexSupplier);
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #setReferenceFrame(ReferenceFrame)} using the reference frame of the first vertex.
    * <li>{@link #set(FrameVertex2DSupplier)}.
    * </ol>
    * <p>
    * If the supplier has no vertex, this polygon is cleared and the reference frame is not updated.
    * </p>
    * 
    * @param frameVertex2DSupplier the supplier of vertices.
    * @throws ReferenceFrameMismatchException if the reference frame of the supplied vertices are
    *            inconsistent.
    * @see FrameVertex2DSupplier
    * @see #set(FrameVertex2DSupplier)
    */
   default void setIncludingFrame(FrameVertex2DSupplier frameVertex2DSupplier)
   {
      if (frameVertex2DSupplier.getNumberOfVertices() == 0)
      {
         clearAndUpdate();
         return;
      }
      else
      {
         setReferenceFrame(frameVertex2DSupplier.getVertex(0).getReferenceFrame());
         set(frameVertex2DSupplier);
      }
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #setReferenceFrame(ReferenceFrame)} using the reference frame of the first vertex.
    * <li>{@link #set(FrameVertex3DSupplier)}.
    * </ol>
    * <p>
    * If the supplier has no vertex, this polygon is cleared and the reference frame is not updated.
    * </p>
    * 
    * @param frameVertex3DSupplier the supplier of vertices.
    * @throws ReferenceFrameMismatchException if the reference frame of the supplied vertices are
    *            inconsistent.
    * @see FrameVertex3DSupplier
    * @see #set(FrameVertex3DSupplier)
    */
   default void setIncludingFrame(FrameVertex3DSupplier frameVertex3DSupplier)
   {
      if (frameVertex3DSupplier.getNumberOfVertices() == 0)
      {
         clearAndUpdate();
         return;
      }
      else
      {
         setReferenceFrame(frameVertex3DSupplier.getVertex(0).getReferenceFrame());
         set(frameVertex3DSupplier);
      }
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #setReferenceFrame(ReferenceFrame)} using the reference frame of
    * {@code firstPolygon}.
    * <li>{@link #set(FrameVertex2DSupplier, FrameVertex2DSupplier)}.
    * </ol>
    * 
    * @param firstVertex2DSupplier the first supplier of vertices.
    * @param secondVertex2DSupplier the second supplier of vertices.
    * @throws ReferenceFrameMismatchException if the reference frame of the supplied vertices are
    *            inconsistent.
    * @see #set(FrameVertex2DSupplier, FrameVertex2DSupplier)
    */
   default void setIncludingFrame(FrameVertex2DSupplier firstVertex2DSupplier, FrameVertex2DSupplier secondVertex2DSupplier)
   {
      if (firstVertex2DSupplier.getNumberOfVertices() > 0)
      {
         setReferenceFrame(firstVertex2DSupplier.getVertex(0).getReferenceFrame());
         set(firstVertex2DSupplier, secondVertex2DSupplier);
      }
      else if (secondVertex2DSupplier.getNumberOfVertices() > 0)
      {
         setReferenceFrame(secondVertex2DSupplier.getVertex(0).getReferenceFrame());
         set(firstVertex2DSupplier, secondVertex2DSupplier);
      }
      else
      {
         clearAndUpdate();
      }
   }

   /**
    * Performs a transformation of the polygon such that it is expressed in a new frame
    * {@code desireFrame}.
    * <p>
    * Because the transformation between two reference frames is a 3D transformation, the result of
    * transforming this polygon's vertices 2D can result in vertices 3D. This method projects the
    * result of the transformation onto the XY-plane.
    * </p>
    *
    * @param desiredFrame the reference frame in which the polygon is to be expressed.
    */
   void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame);
}
