package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Write and read interface for a convex polygon defined in the XY-plane and that is expressed in an
 * immutable reference frame.
 * <p>
 * This implementation of convex polygon is designed for garbage free operations.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FixedFrameConvexPolygon2DBasics extends FrameConvexPolygon2DReadOnly, ConvexPolygon2DBasics
{
   /**
    * Gets the reference to the {@code index}<sup>th</sup> vertex of this polygon.
    *
    * @param index the index of the vertex in the clockwise ordered list.
    * @return the reference to the vertex.
    * @see #getVertexUnsafe(int)
    */
   @Override
   FixedFramePoint2DBasics getVertexUnsafe(int index);

   /**
    * Add a vertex to this polygon.
    *
    * @param referenceFrame the reference frame in which the given vertex is expressed.
    * @param x the x-coordinate of the new vertex.
    * @param y the y-coordinate of the new vertex.
    * @throws ReferenceFrameMismatchException if {@code referenceFrame} and
    *            {@code this.getReferenceFrame()} are not the same.
    * @see #addVertex(double, double)
    */
   default void addVertex(ReferenceFrame referenceFrame, double x, double y)
   {
      checkReferenceFrameMatch(referenceFrame);
      addVertex(x, y);
   }

   /**
    * Add a vertex to this polygon.
    *
    * @param referenceFrame the reference frame in which the given {@code vertex} is expressed.
    * @param vertex the new vertex. Not modified.
    * @throws ReferenceFrameMismatchException if {@code referenceFrame} and
    *            {@code this.getReferenceFrame()} are not the same.
    * @see #addVertex(Point2DReadOnly)
    */
   default void addVertex(ReferenceFrame referenceFrame, Point2DReadOnly vertex)
   {
      checkReferenceFrameMatch(referenceFrame);
      addVertex(vertex);
   }

   /**
    * Add a vertex to this polygon.
    * <p>
    * If {@code vertex} is expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #addVertex(Point2DReadOnly)}.
    * </p>
    * <p>
    * If {@code vertex} is expressed in a different frame than {@code this}, then the {@code vertex}
    * is added once transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the given {@code vertex} is expressed.
    * @param vertex the new vertex. Not modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of
    *           the transform from the vertex's frame to this frame represents a transformation in
    *           the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the rotation
    *            part of the transform is not a transformation in the XY plane.
    * @see #addVertex(Point2DReadOnly)
    */
   void addVertexMatchingFrame(ReferenceFrame referenceFrame, Point2DReadOnly vertex, boolean checkIfTransformInXYPlane);

   /**
    * Add a vertex to this polygon using only the x and y coordinates of the given {@code vertex}.
    *
    * @param referenceFrame the reference frame in which the given {@code vertex} is expressed.
    * @param vertex the new vertex. Not modified.
    * @throws ReferenceFrameMismatchException if {@code referenceFrame} and
    *            {@code this.getReferenceFrame()} are not the same.
    * @see #addVertex(Point3DReadOnly)
    */
   default void addVertex(ReferenceFrame referenceFrame, Point3DReadOnly vertex)
   {
      checkReferenceFrameMatch(referenceFrame);
      addVertex(vertex);
   }

   /**
    * Add a vertex to this polygon.
    * <p>
    * If {@code vertex} is expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #addVertex(Point2DReadOnly)}.
    * </p>
    * <p>
    * If {@code vertex} is expressed in a different frame than {@code this}, then the {@code vertex}
    * is added once transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame the reference frame in which the given {@code vertex} is expressed.
    * @param vertex the new vertex. Not modified.
    * @see #addVertex(Point3DReadOnly)
    */
   void addVertexMatchingFrame(ReferenceFrame referenceFrame, Point3DReadOnly vertex);

   /**
    * Add a vertex to this polygon.
    *
    * @param vertex the new vertex. Not modified.
    * @throws ReferenceFrameMismatchException if {@code vertex} and {@code this} are not expressed
    *            in the same reference frame.
    * @see #addVertex(Point2DReadOnly)
    */
   default void addVertex(FramePoint2DReadOnly vertex)
   {
      checkReferenceFrameMatch(vertex);
      ConvexPolygon2DBasics.super.addVertex(vertex);
   }

   /**
    * Add a vertex to this polygon.
    * <p>
    * If {@code vertex} is expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #addVertex(Point2DReadOnly)}.
    * </p>
    * <p>
    * If {@code vertex} is expressed in a different frame than {@code this}, then the {@code vertex}
    * is added once transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param vertex the new vertex. Not modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of
    *           the transform from the vertex's frame to this frame represents a transformation in
    *           the XY plane.
    * @see #addVertexMatchingFrame(ReferenceFrame, Point2DReadOnly, boolean)
    * @see #addVertex(Point2DReadOnly)
    */
   default void addVertexMatchingFrame(FramePoint2DReadOnly vertex, boolean checkIfTransformInXYPlane)
   {
      addVertexMatchingFrame(vertex.getReferenceFrame(), vertex, checkIfTransformInXYPlane);
   }

   /**
    * Add a vertex to this polygon using only the x and y coordinates of the given {@code vertex}.
    *
    * @param vertex the new vertex. Not modified.
    * @throws ReferenceFrameMismatchException if {@code vertex} and {@code this} are not expressed
    *            in the same reference frame.
    * @see #addVertex(Point3DReadOnly)
    */
   default void addVertex(FramePoint3DReadOnly vertex)
   {
      checkReferenceFrameMatch(vertex);
      ConvexPolygon2DBasics.super.addVertex(vertex);
   }

   /**
    * Add a vertex to this polygon using only the x and y coordinates of the given {@code vertex}.
    *
    * @param vertex the new vertex. Not modified.
    * @see #addVertexMatchingFrame(ReferenceFrame, Point3DReadOnly)
    * @see #addVertex(Point3DReadOnly)
    */
   default void addVertexMatchingFrame(FramePoint3DReadOnly vertex)
   {
      addVertexMatchingFrame(vertex.getReferenceFrame(), vertex);
   }

   /**
    * Adds the vertices from the given vertex supplier.
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    *
    * @param frameVertex2DSupplier the supplier of vertices.
    * @throws ReferenceFrameMismatchException if any of the given {@code vertices} is a
    *            {@code FramePoint2DReadOnly} and is not expressed in the same reference frame as
    *            {@code this}.
    * @see FrameVertex2DSupplier
    * @see #addVertex(FramePoint2DReadOnly)
    */
   default void addVertices(FrameVertex2DSupplier frameVertex2DSupplier)
   {
      for (int index = 0; index < frameVertex2DSupplier.getNumberOfVertices(); index++)
      {
         addVertex(frameVertex2DSupplier.getVertex(index));
      }
   }

   /**
    * Adds the vertices from the given vertex supplier.
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    *
    * @param frameVertex2DSupplier the supplier of vertices.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of
    *           the transform from the vertex's frame to this frame represents a transformation in
    *           the XY plane.
    * @see FrameVertex2DSupplier
    * @see #addVertexMatchingFrame(FramePoint2DReadOnly, boolean)
    */
   default void addVerticesMatchingFrame(FrameVertex2DSupplier frameVertex2DSupplier, boolean checkIfTransformInXYPlane)
   {
      for (int index = 0; index < frameVertex2DSupplier.getNumberOfVertices(); index++)
      {
         addVertexMatchingFrame(frameVertex2DSupplier.getVertex(index), checkIfTransformInXYPlane);
      }
   }

   /**
    * Adds the vertices from the given vertex supplier.
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    * <p>
    * Only the x and y coordinates of each vertex is used to add a vertex to this polygon.
    * </p>
    *
    * @param frameVertex3DSupplier the supplier of vertices.
    * @throws ReferenceFrameMismatchException if any of the given {@code vertices} is a
    *            {@code FramePoint3DReadOnly} and is not expressed in the same reference frame as
    *            {@code this}.
    * @see FrameVertex3DSupplier
    * @see #addVertex(FramePoint3DReadOnly)
    */
   default void addVertices(FrameVertex3DSupplier frameVertex3DSupplier)
   {
      for (int index = 0; index < frameVertex3DSupplier.getNumberOfVertices(); index++)
      {
         addVertex(frameVertex3DSupplier.getVertex(index));
      }
   }

   /**
    * Adds the vertices from the given vertex supplier.
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    *
    * @param frameVertex3DSupplier the supplier of vertices.
    * @see FrameVertex3DSupplier
    * @see #addVertexMatchingFrame(FramePoint3DReadOnly)
    */
   default void addVerticesMatchingFrame(FrameVertex3DSupplier frameVertex3DSupplier)
   {
      for (int index = 0; index < frameVertex3DSupplier.getNumberOfVertices(); index++)
      {
         addVertexMatchingFrame(frameVertex3DSupplier.getVertex(index));
      }
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #clear()}.
    * <li>{@link #addVertices(FrameVertex2DSupplier)}.
    * <li>{@link #update()}.
    * </ol>
    *
    * @param frameVertex2DSupplier the supplier of vertices.
    * @throws ReferenceFrameMismatchException if any of the given {@code vertices} is a
    *            {@code FramePoint2DReadOnly} and is not expressed in the same reference frame as
    *            {@code this}.
    * @see FrameVertex2DSupplier
    * @see #addVertices(FrameVertex2DSupplier)
    */
   default void set(FrameVertex2DSupplier frameVertex2DSupplier)
   {
      clear();
      addVertices(frameVertex2DSupplier);
      update();
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #clear()}.
    * <li>{@link #addVerticesMatchingFrame(FrameVertex2DSupplier, boolean)}.
    * <li>{@link #update()}.
    * </ol>
    *
    * @param frameVertex2DSupplier the supplier of vertices.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of
    *           the transform from the vertex's frame to this frame represents a transformation in
    *           the XY plane.
    * @see FrameVertex2DSupplier
    * @see #addVerticesMatchingFrame(FrameVertex2DSupplier, boolean)
    */
   default void setMatchingFrame(FrameVertex2DSupplier frameVertex2DSupplier, boolean checkIfTransformInXYPlane)
   {
      clear();
      addVerticesMatchingFrame(frameVertex2DSupplier, checkIfTransformInXYPlane);
      update();
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #clear()}.
    * <li>{@link #addVertices(FrameVertex3DSupplier)}.
    * <li>{@link #update()}.
    * </ol>
    *
    * @param frameVertex3DSupplier the supplier of vertices.
    * @throws ReferenceFrameMismatchException if any of the given {@code vertices} is a
    *            {@code FramePoint3DReadOnly} and is not expressed in the same reference frame as
    *            {@code this}.
    * @see FrameVertex3DSupplier
    * @see #addVertices(FrameVertex3DSupplier)
    */
   default void set(FrameVertex3DSupplier frameVertex3DSupplier)
   {
      clear();
      addVertices(frameVertex3DSupplier);
      update();
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #clear()}.
    * <li>{@link #addVertices(FrameVertex3DSupplier)}.
    * <li>{@link #update()}.
    * </ol>
    *
    * @param frameVertex3DSupplier the supplier of vertices.
    * @see FrameVertex3DSupplier
    * @see #addVerticesMatchingFrame(FrameVertex3DSupplier)
    */
   default void setMatchingFrame(FrameVertex3DSupplier frameVertex3DSupplier)
   {
      clear();
      addVerticesMatchingFrame(frameVertex3DSupplier);
      update();
   }

   /**
    * Sets this polygon such that it represents the smallest convex hull that contains both
    * polygons.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    *
    * @param firstVertex2DSupplier the first supplier of vertices.
    * @param secondVertex2DSupplier the second supplier of vertices.
    * @throws ReferenceFrameMismatchException if any of the vertices provided by
    *            {@code firstVertexSupplier} and {@code secondVertexSupplier} are not expressed in
    *            the same reference frame as {@code this}.
    * @see FrameVertex2DSupplier
    * @see #addVertices(FrameVertex2DSupplier)
    */
   default void set(FrameVertex2DSupplier firstVertex2DSupplier, FrameVertex2DSupplier secondVertex2DSupplier)
   {
      clear();
      addVertices(firstVertex2DSupplier);
      addVertices(secondVertex2DSupplier);
      update();
   }

   /**
    * Sets this polygon such that it represents the smallest convex hull that contains both
    * polygons.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    *
    * @param firstVertex2DSupplier the first supplier of vertices.
    * @param secondVertex2DSupplier the second supplier of vertices.
    * @throws ReferenceFrameMismatchException if any of the vertices provided by
    *            {@code firstVertexSupplier} are not expressed in the same reference frame as
    *            {@code this}.
    * @see FrameVertex2DSupplier
    * @see #addVertices(FrameVertex2DSupplier)
    */
   default void set(FrameVertex2DSupplier firstVertex2DSupplier, Vertex2DSupplier secondVertex2DSupplier)
   {
      clear();
      addVertices(firstVertex2DSupplier);
      addVertices(secondVertex2DSupplier);
      update();
   }

   /**
    * Sets this polygon such that it represents the smallest convex hull that contains both
    * polygons.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    *
    * @param firstVertex2DSupplier the first supplier of vertices.
    * @param secondVertex2DSupplier the second supplier of vertices.
    * @throws ReferenceFrameMismatchException if any of the vertices provided by
    *            {@code secondVertexSupplier} are not expressed in the same reference frame as
    *            {@code this}.
    * @see FrameVertex2DSupplier
    * @see #addVertices(FrameVertex2DSupplier)
    */
   default void set(Vertex2DSupplier firstVertex2DSupplier, FrameVertex2DSupplier secondVertex2DSupplier)
   {
      clear();
      addVertices(firstVertex2DSupplier);
      addVertices(secondVertex2DSupplier);
      update();
   }

   /**
    * Scale this convex polygon about {@code pointToScaleAbout}.
    *
    * @param pointToScaleAbout the center of the scale transformation. Not modified.
    *
    * @param scaleFactor the scale factor to apply to this polygon. A value of {@code 1.0} does
    *           nothing.
    * @throws ReferenceFrameMismatchException if {@code pointToScaleAbout} and {@code this} are not
    *            expressed in the same reference frame.
    * @see #scale(Point2DReadOnly, double)
    */
   default void scale(FramePoint2DReadOnly pointToScaleAbout, double scaleFactor)
   {
      checkReferenceFrameMatch(pointToScaleAbout);
      ConvexPolygon2DBasics.super.scale(pointToScaleAbout, scaleFactor);
   }

   /**
    * Translates this polygon.
    *
    * @param translation the translation to apply to this polygon's vertices. Not modified.
    * @throws ReferenceFrameMismatchException if {@code translation} and {@code this} are not
    *            expressed in the same reference frame.
    * @see #translate(Tuple2DReadOnly)
    */
   default void translate(FrameTuple2DReadOnly translation)
   {
      checkReferenceFrameMatch(translation);
      ConvexPolygon2DBasics.super.translate(translation);
   }
}
