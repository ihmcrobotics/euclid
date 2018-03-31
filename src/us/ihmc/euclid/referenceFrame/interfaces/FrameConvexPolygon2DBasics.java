package us.ihmc.euclid.referenceFrame.interfaces;

import java.util.List;

import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

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
    * <li>{@link #setAndUpdate(List, int)}.
    * </ol>
    * 
    * @param referenceFrame the new reference frame for this frame convex polygon.
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the list. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @see #setAndUpdate(List, int)
    */
   default void setIncludingFrameAndUpdate(ReferenceFrame referenceFrame, List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {
      setReferenceFrame(referenceFrame);
      setAndUpdate(vertices, numberOfVertices);
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #setReferenceFrame(ReferenceFrame)}
    * <li>{@link #setAndUpdate3D(List, int)}.
    * </ol>
    * 
    * @param referenceFrame the new reference frame for this frame convex polygon.
    * @param vertices the 3D point cloud from which the convex hull is to be computed. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the list. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @see #setAndUpdate3D(List, int)
    */
   default void setIncludingFrameAndUpdate3D(ReferenceFrame referenceFrame, List<? extends Point3DReadOnly> vertices, int numberOfVertices)
   {
      setReferenceFrame(referenceFrame);
      setAndUpdate3D(vertices, numberOfVertices);
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #setReferenceFrame(ReferenceFrame)}
    * <li>{@link #setAndUpdate(Point2DReadOnly[], int)}.
    * </ol>
    * 
    * @param referenceFrame the new reference frame for this frame convex polygon.
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the array. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @see #setAndUpdate(Point2DReadOnly[], int)
    */
   default void setIncludingFrameAndUpdate(ReferenceFrame referenceFrame, Point2DReadOnly[] vertices, int numberOfVertices)
   {
      setReferenceFrame(referenceFrame);
      setAndUpdate(vertices, numberOfVertices);
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #setReferenceFrame(ReferenceFrame)}
    * <li>{@link #setAndUpdate(Point2DReadOnly[], int)}.
    * </ol>
    * 
    * @param referenceFrame the new reference frame for this frame convex polygon.
    * @param vertices the 3D point cloud from which the convex hull is to be computed. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the array. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @see #setAndUpdate(Point3DReadOnly[], int)
    */
   default void setIncludingFrameAndUpdate(ReferenceFrame referenceFrame, Point3DReadOnly[] vertices, int numberOfVertices)
   {
      setReferenceFrame(referenceFrame);
      setAndUpdate(vertices, numberOfVertices);
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #setReferenceFrame(ReferenceFrame)}
    * <li>{@link #setAndUpdate(ConvexPolygon2DReadOnly)}.
    * </ol>
    * 
    * @param referenceFrame the new reference frame for this frame convex polygon.
    * @param other the other convex polygon to copy. Not modified.
    * @see #setAndUpdate(ConvexPolygon2DReadOnly)
    */
   default void setIncludingFrameAndUpdate(ReferenceFrame referenceFrame, ConvexPolygon2DReadOnly other)
   {
      setReferenceFrame(referenceFrame);
      setAndUpdate(other);
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #setReferenceFrame(ReferenceFrame)}
    * <li>{@link #setAndUpdate(ConvexPolygon2DReadOnly, ConvexPolygon2DReadOnly)}.
    * </ol>
    * 
    * @param referenceFrame the new reference frame for this frame convex polygon.
    * @param firstPolygon the first convex polygon to combine. Not modified.
    * @param secondPolygon the second convex polygon to combine. Not modified.
    * @see #setAndUpdate(ConvexPolygon2DReadOnly, ConvexPolygon2DReadOnly)
    */
   default void setIncludingFrameAndUpdate(ReferenceFrame referenceFrame, ConvexPolygon2DReadOnly firstPolygon, ConvexPolygon2DReadOnly secondPolygon)
   {
      setReferenceFrame(referenceFrame);
      setAndUpdate(firstPolygon, secondPolygon);
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #setReferenceFrame(ReferenceFrame)} using the reference frame of the first vertex.
    * <li>{@link #setAndUpdate(List, int)}.
    * </ol>
    * 
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the list. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @see #setAndUpdate(List, int)
    */
   default void setIncludingFrameAndUpdate(List<? extends FramePoint2DReadOnly> vertices, int numberOfVertices)
   {
      setReferenceFrame(vertices.get(0).getReferenceFrame());
      setAndUpdate(vertices, numberOfVertices);
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #setReferenceFrame(ReferenceFrame)} using the reference frame of the first vertex.
    * <li>{@link #setAndUpdate(List, int)}.
    * </ol>
    * 
    * @param vertices the 3D point cloud from which the convex hull is to be computed. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the list. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @see #setAndUpdate3D(List, int)
    */
   default void setIncludingFrameAndUpdate3D(List<? extends FramePoint3DReadOnly> vertices, int numberOfVertices)
   {
      setReferenceFrame(vertices.get(0).getReferenceFrame());
      setAndUpdate3D(vertices, numberOfVertices);
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #setReferenceFrame(ReferenceFrame)} using the reference frame of the first vertex.
    * <li>{@link #setAndUpdate(FramePoint2DReadOnly[], int)}.
    * </ol>
    * 
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the array. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @see #setAndUpdate(FramePoint2DReadOnly[], int)
    */
   default void setIncludingFrameAndUpdate(FramePoint2DReadOnly[] vertices, int numberOfVertices)
   {
      setReferenceFrame(vertices[0].getReferenceFrame());
      setAndUpdate(vertices, numberOfVertices);
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #setReferenceFrame(ReferenceFrame)} using the reference frame of the first vertex.
    * <li>{@link #setAndUpdate(FramePoint3DReadOnly[], int)}.
    * </ol>
    * 
    * @param vertices the 3D point cloud from which the convex hull is to be computed. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the array. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    */
   default void setIncludingFrameAndUpdate(FramePoint3DReadOnly[] vertices, int numberOfVertices)
   {
      setReferenceFrame(vertices[0].getReferenceFrame());
      setAndUpdate(vertices, numberOfVertices);
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #setReferenceFrame(ReferenceFrame)} using the reference frame of {@code other}.
    * <li>{@link #setAndUpdate(ConvexPolygon2DReadOnly)}.
    * </ol>
    * 
    * @param other the other convex polygon to copy. Not modified.
    * @see #setIncludingFrameAndUpdate(ReferenceFrame, ConvexPolygon2DReadOnly)
    * @see #setAndUpdate(ConvexPolygon2DReadOnly)
    */
   default void setIncludingFrameAndUpdate(FrameConvexPolygon2DReadOnly other)
   {
      setIncludingFrameAndUpdate(other.getReferenceFrame(), other);
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #setReferenceFrame(ReferenceFrame)} using the reference frame of
    * {@code firstPolygon}.
    * <li>{@link #setAndUpdate(ConvexPolygon2DReadOnly, ConvexPolygon2DReadOnly)}.
    * </ol>
    * 
    * @param firstPolygon the first convex polygon to combine. Not modified.
    * @param secondPolygon the second convex polygon to combine. Not modified.
    * @throws ReferenceFrameMismatchException if {@code firstPolygon} and {@code secondPolygon} are
    *            not expressed in the same reference frame.
    * @see #setAndUpdate(ConvexPolygon2DReadOnly, ConvexPolygon2DReadOnly)
    */
   default void setIncludingFrameAndUpdate(FrameConvexPolygon2DReadOnly firstPolygon, FrameConvexPolygon2DReadOnly secondPolygon)
   {
      firstPolygon.checkReferenceFrameMatch(secondPolygon);
      setReferenceFrame(firstPolygon.getReferenceFrame());
      setAndUpdate((ConvexPolygon2DReadOnly) firstPolygon, (ConvexPolygon2DReadOnly) secondPolygon);
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
