package us.ihmc.euclid.referenceFrame.interfaces;

import java.util.List;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
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
    * {@inheritDoc}
    * <p>
    * WARNING: Each element of the given list is tested such that if they implement
    * {@code FramePoint2DReadOnly}, the method checks that they are expressed in the same reference
    * frame as {@code this}.
    * </p>
    * 
    * @throws ReferenceFrameMismatchException if any of the given {@code vertices} is a
    *            {@code FramePoint2DReadOnly} and is not expressed in the same reference frame as
    *            {@code this}.
    */
   @Override
   default void addVertices(List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {
      if (numberOfVertices < 0 || numberOfVertices > vertices.size())
         throw new IllegalArgumentException("Illegal numberOfVertices: " + numberOfVertices + ", expected a value in ] 0, " + vertices.size() + "].");

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2DReadOnly vertex = vertices.get(i);
         if (vertex instanceof FramePoint2DReadOnly)
            addVertex((FramePoint2DReadOnly) vertex);
         else
            addVertex(vertex);
      }
   }

   /**
    * {@inheritDoc}
    * <p>
    * WARNING: Each element of the given list is tested such that if they implement
    * {@code FramePoint3DReadOnly}, the method checks that they are expressed in the same reference
    * frame as {@code this}.
    * </p>
    * 
    * @throws ReferenceFrameMismatchException if any of the given {@code vertices} is a
    *            {@code FramePoint3DReadOnly} and is not expressed in the same reference frame as
    *            {@code this}.
    */
   @Override
   default void addVertices3D(List<? extends Point3DReadOnly> vertices, int numberOfVertices)
   {
      if (numberOfVertices < 0 || numberOfVertices > vertices.size())
         throw new IllegalArgumentException("Illegal numberOfVertices: " + numberOfVertices + ", expected a value in ] 0, " + vertices.size() + "].");

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point3DReadOnly vertex = vertices.get(i);
         if (vertex instanceof FramePoint2DReadOnly)
            addVertex((FramePoint2DReadOnly) vertex);
         else
            addVertex(vertex);
      }
   }

   /**
    * Adds the N first vertices from the given array to this polygon, where N is equal to
    * {@code numberOfVertices}.
    *
    * @param vertices the array containing the vertices to add to this polygon. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the array. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @throws ReferenceFrameMismatchException if any of the given {@code vertices} is not expressed
    *            in the same reference frame as {@code this}.
    * @see #addVertices(Point2DReadOnly[], int)
    */
   default void addVertices(FramePoint2DReadOnly[] vertices, int numberOfVertices)
   {
      if (numberOfVertices < 0 || numberOfVertices > vertices.length)
         throw new IllegalArgumentException("Illegal numberOfVertices: " + numberOfVertices + ", expected a value in ] 0, " + vertices.length + "].");

      for (int i = 0; i < numberOfVertices; i++)
         addVertex(vertices[i]);
   }

   /**
    * Adds the N first vertices from the given array to this polygon, where N is equal to
    * {@code numberOfVertices}.
    *
    * @param vertices the array containing the vertices to add to this polygon. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the array. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @throws ReferenceFrameMismatchException if any of the given {@code vertices} is not expressed
    *            in the same reference frame as {@code this}.
    * @see #addVertices(Point3DReadOnly[], int)
    */
   default void addVertices(FramePoint3DReadOnly[] vertices, int numberOfVertices)
   {
      if (numberOfVertices < 0 || numberOfVertices > vertices.length)
         throw new IllegalArgumentException("Illegal numberOfVertices: " + numberOfVertices + ", expected a value in ] 0, " + vertices.length + "].");

      for (int i = 0; i < numberOfVertices; i++)
         addVertex(vertices[i]);
   }

   /**
    * Adds new vertices to this polygon from another convex polygon.
    *
    * @param otherPolygon the other convex polygon that is used to add new vertices to this polygon.
    *           Not modified.
    * @see #addVertices(ConvexPolygon2DReadOnly)
    */
   default void addVertices(FrameConvexPolygon2DReadOnly otherPolygon)
   {
      for (int i = 0; i < otherPolygon.getNumberOfVertices(); i++)
         addVertex(otherPolygon.getVertex(i));
   }

   /**
    * {@inheritDoc}
    * <p>
    * WARNING: Each element of the given list is tested such that if they implement
    * {@code FramePoint2DReadOnly}, the method checks that they are expressed in the same reference
    * frame as {@code this}.
    * </p>
    * 
    * @throws ReferenceFrameMismatchException if any of the given {@code vertices} is a
    *            {@code FramePoint2DReadOnly} and is not expressed in the same reference frame as
    *            {@code this}.
    */
   @Override
   default void setAndUpdate(List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {
      clear();
      addVertices(vertices, numberOfVertices);
      update();
   }

   /**
    * {@inheritDoc}
    * <p>
    * WARNING: Each element of the given list is tested such that if they implement
    * {@code FramePoint3DReadOnly}, the method checks that they are expressed in the same reference
    * frame as {@code this}.
    * </p>
    *
    * @throws ReferenceFrameMismatchException if any of the given {@code vertices} is a
    *            {@code FramePoint3DReadOnly} and is not expressed in the same reference frame as
    *            {@code this}.
    */
   @Override
   default void setAndUpdate3D(List<? extends Point3DReadOnly> vertices, int numberOfVertices)
   {
      clear();
      addVertices3D(vertices, numberOfVertices);
      update();
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #clear()}.
    * <li>{@link #addVertices(FramePoint2DReadOnly[], int)}.
    * <li>{@link #update()}.
    * </ol>
    *
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the array. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @throws ReferenceFrameMismatchException if any of the given {@code vertices} is not expressed
    *            in the same reference frame as {@code this}.
    * @see #setAndUpdate(Point2DReadOnly[], int)
    */
   default void setAndUpdate(FramePoint2DReadOnly[] vertices, int numberOfVertices)
   {
      clear();
      addVertices(vertices, numberOfVertices);
      update();
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #clear()}.
    * <li>{@link #addVertices(FramePoint3DReadOnly[], int)}.
    * <li>{@link #update()}.
    * </ol>
    *
    * @param vertices the 3D point cloud from which the convex hull is to be computed. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the array. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @throws ReferenceFrameMismatchException if any of the given {@code vertices} is not expressed
    *            in the same reference frame as {@code this}.
    * @see #setAndUpdate(Point3DReadOnly[], int)
    */
   default void setAndUpdate(FramePoint3DReadOnly[] vertices, int numberOfVertices)
   {
      clear();
      addVertices(vertices, numberOfVertices);
      update();
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #clear()}.
    * <li>{@link #addVertices(FrameConvexPolygon2DReadOnly)}.
    * <li>{@link #update()}.
    * </ol>
    *
    * @param other the other convex polygon to copy. Not modified.
    * @see #setAndUpdate(ConvexPolygon2DReadOnly)
    */
   default void setAndUpdate(FrameConvexPolygon2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      ConvexPolygon2DBasics.super.setAndUpdate(other);
   }

   /**
    * Sets this polygon such that it represents the smallest convex hull that contains both
    * polygons.
    *
    * @param firstPolygon the first convex polygon to combine. Not modified.
    * @param secondPolygon the second convex polygon to combine. Not modified.
    * @throws ReferenceFrameMismatchException if {@code firstPolygon}, {@code secondPolygon}, and
    *            {@code this} are not expressed in the same reference frame.
    * @see #setAndUpdate(ConvexPolygon2DReadOnly, ConvexPolygon2DReadOnly)
    */
   default void setAndUpdate(FrameConvexPolygon2DReadOnly firstPolygon, FrameConvexPolygon2DReadOnly secondPolygon)
   {
      checkReferenceFrameMatch(firstPolygon);
      checkReferenceFrameMatch(secondPolygon);
      ConvexPolygon2DBasics.super.setAndUpdate(firstPolygon, secondPolygon);
   }

   /**
    * Sets this polygon such that it represents the smallest convex hull that contains both
    * polygons.
    *
    * @param firstPolygon the first convex polygon to combine. Not modified.
    * @param secondPolygon the second convex polygon to combine. Not modified.
    * @throws ReferenceFrameMismatchException if {@code firstPolygon} and {@code this} are not
    *            expressed in the same reference frame.
    * @see #setAndUpdate(ConvexPolygon2DReadOnly, ConvexPolygon2DReadOnly)
    */
   default void setAndUpdate(FrameConvexPolygon2DReadOnly firstPolygon, ConvexPolygon2DReadOnly secondPolygon)
   {
      checkReferenceFrameMatch(firstPolygon);
      ConvexPolygon2DBasics.super.setAndUpdate(firstPolygon, secondPolygon);
   }

   /**
    * Sets this polygon such that it represents the smallest convex hull that contains both
    * polygons.
    *
    * @param firstPolygon the first convex polygon to combine. Not modified.
    * @param secondPolygon the second convex polygon to combine. Not modified.
    * @throws ReferenceFrameMismatchException if {@code secondPolygon} and {@code this} are not
    *            expressed in the same reference frame.
    * @see #setAndUpdate(ConvexPolygon2DReadOnly, ConvexPolygon2DReadOnly)
    */
   default void setAndUpdate(ConvexPolygon2DReadOnly firstPolygon, FrameConvexPolygon2DReadOnly secondPolygon)
   {
      checkReferenceFrameMatch(secondPolygon);
      ConvexPolygon2DBasics.super.setAndUpdate(firstPolygon, secondPolygon);
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
