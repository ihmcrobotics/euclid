package us.ihmc.euclid.referenceFrame.interfaces;

import java.util.List;

import us.ihmc.euclid.geometry.exceptions.EmptyPolygonException;
import us.ihmc.euclid.geometry.exceptions.OutdatedPolygonException;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
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
    * <p>
    * WARNING: Through this method, the user can modify the vertices of this polygon without it being
    * notified of such change. When modifying a vertex, the method {@link #notifyVerticesChanged()}
    * should be called followed by {@link #update()}. Always prefer using {@link #getVertex(int)}
    * instead.
    * </p>
    * <p>
    * Note that this polygon's vertices are clockwise ordered and that the first vertex has the lowest
    * x-coordinate.
    * </p>
    *
    * @param index the index of the vertex in the clockwise ordered list.
    * @return the reference to the vertex.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   @Override
   FixedFramePoint2DBasics getVertexUnsafe(int index);

   /**
    * Add a vertex to this polygon.
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    *
    * @param referenceFrame the reference frame in which the given {@code vertex} is expressed.
    * @param vertex the new vertex. Not modified.
    * @throws ReferenceFrameMismatchException if {@code referenceFrame} and
    *            {@code this.getReferenceFrame()} are not the same.
    */
   default void addVertex(ReferenceFrame referenceFrame, Point2DReadOnly vertex)
   {
      checkReferenceFrameMatch(referenceFrame);
      addVertex(vertex);
   }

   /**
    * Add a vertex to this polygon using only the x and y coordinates of the given {@code vertex}.
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    *
    * @param referenceFrame the reference frame in which the given {@code vertex} is expressed.
    * @param vertex the new vertex. Not modified.
    * @throws ReferenceFrameMismatchException if {@code referenceFrame} and
    *            {@code this.getReferenceFrame()} are not the same.
    */
   default void addVertex(ReferenceFrame referenceFrame, Point3DReadOnly vertex)
   {
      checkReferenceFrameMatch(referenceFrame);
      addVertex(vertex);
   }

   /**
    * Add a vertex to this polygon.
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    *
    * @param vertex the new vertex. Not modified.
    * @throws ReferenceFrameMismatchException if {@code vertex} and {@code this} are not expressed in
    *            the same reference frame.
    */
   default void addVertex(FramePoint2DReadOnly vertex)
   {
      checkReferenceFrameMatch(vertex);
      ConvexPolygon2DBasics.super.addVertex(vertex);
   }

   /**
    * Add a vertex to this polygon using only the x and y coordinates of the given {@code vertex}.
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    *
    * @param vertex the new vertex. Not modified.
    * @throws ReferenceFrameMismatchException if {@code vertex} and {@code this} are not expressed in
    *            the same reference frame.
    */
   default void addVertex(FramePoint3DReadOnly vertex)
   {
      checkReferenceFrameMatch(vertex);
      ConvexPolygon2DBasics.super.addVertex(vertex);
   }

   /**
    * Adds the N first vertices from the given list to this polygon, where N is equal to
    * {@code numberOfVertices}.
    * <p>
    * WARNING: Each element of the given list is tested such that if they implement
    * {@code FramePoint2DReadOnly}, the method checks that they are expressed in the same reference
    * frame as {@code this}.
    * </p>
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    *
    * @param vertices the list containing the vertices to add to this polygon. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the list. Only the points &in;
    *           [0; {@code numberOfVertices}[ are processed.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the size
    *            of the given list of vertices.
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
    * Adds the N first vertices from the given list to this polygon, where N is equal to
    * {@code numberOfVertices}.
    * <p>
    * Only the x and y coordinates of each vertex is used to add a vertex to this polygon.
    * </p>
    * <p>
    * WARNING: Each element of the given list is tested such that if they implement
    * {@code FramePoint3DReadOnly}, the method checks that they are expressed in the same reference
    * frame as {@code this}.
    * </p>
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    *
    * @param vertices the list containing the vertices to add to this polygon. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the list. Only the points &in;
    *           [0; {@code numberOfVertices}[ are processed.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the size
    *            of the given list of vertices.
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
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    *
    * @param vertices the array containing the vertices to add to this polygon. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the array. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the size
    *            of the given array of vertices.
    * @throws ReferenceFrameMismatchException if any of the given {@code vertices} is not expressed in
    *            the same reference frame as {@code this}.
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
    * <p>
    * Only the x and y coordinates of each vertex is used to add a vertex to this polygon.
    * </p>
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    *
    * @param vertices the array containing the vertices to add to this polygon. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the array. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the size
    *            of the given array of vertices.
    * @throws ReferenceFrameMismatchException if any of the given {@code vertices} is not expressed in
    *            the same reference frame as {@code this}.
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
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    *
    * @param otherPolygon the other convex polygon that is used to add new vertices to this polygon.
    *           Not modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time the
    *            other polygon's vertices were edited.
    */
   default void addVertices(FrameConvexPolygon2DReadOnly otherPolygon)
   {
      for (int i = 0; i < otherPolygon.getNumberOfVertices(); i++)
         addVertex(otherPolygon.getVertex(i));
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #clear()}.
    * <li>{@link #addVertices(List, int)}.
    * <li>{@link #update()}.
    * </ol>
    * <p>
    * WARNING: Each element of the given list is tested such that if they implement
    * {@code FramePoint2DReadOnly}, the method checks that they are expressed in the same reference
    * frame as {@code this}.
    * </p>
    *
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the list. Only the points &in;
    *           [0; {@code numberOfVertices}[ are processed.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the size
    *            of the given list of vertices.
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
    * This method does:
    * <ol>
    * <li>{@link #clear()}.
    * <li>{@link #addVertices3D(List, int)}.
    * <li>{@link #update()}.
    * </ol>
    * <p>
    * WARNING: Each element of the given list is tested such that if they implement
    * {@code FramePoint3DReadOnly}, the method checks that they are expressed in the same reference
    * frame as {@code this}.
    * </p>
    *
    * @param vertices the 3D point cloud from which the convex hull is to be computed. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the list. Only the points &in;
    *           [0; {@code numberOfVertices}[ are processed.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the size
    *            of the given list of vertices.
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
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the size
    *            of the given array of vertices.
    * @throws ReferenceFrameMismatchException if any of the given {@code vertices} is not expressed in
    *            the same reference frame as {@code this}.
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
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the size
    *            of the given array of vertices.
    * @throws ReferenceFrameMismatchException if any of the given {@code vertices} is not expressed in
    *            the same reference frame as {@code this}.
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
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time the
    *            other polygon's vertices were edited.
    */
   // TODO There is no need to call update() there, instead update everything from the other polygon to make it faster.
   default void setAndUpdate(FrameConvexPolygon2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      ConvexPolygon2DBasics.super.setAndUpdate(other);
   }

   /**
    * Sets this polygon such that it represents the smallest convex hull that contains both polygons.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    *
    * @param firstPolygon the first convex polygon to combine. Not modified.
    * @param secondPolygon the second convex polygon to combine. Not modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time the
    *            other polygons' vertices were edited.
    * @throws ReferenceFrameMismatchException if {@code firstPolygon}, {@code secondPolygon}, and
    *            {@code this} are not expressed in the same reference frame.
    */
   // TODO: Make this more efficient by finding the rotating calipers, as in the intersection method.
   default void setAndUpdate(FrameConvexPolygon2DReadOnly firstPolygon, FrameConvexPolygon2DReadOnly secondPolygon)
   {
      checkReferenceFrameMatch(firstPolygon);
      checkReferenceFrameMatch(secondPolygon);
      ConvexPolygon2DBasics.super.setAndUpdate(firstPolygon, secondPolygon);
   }

   /**
    * Sets this polygon such that it represents the smallest convex hull that contains both polygons.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    *
    * @param firstPolygon the first convex polygon to combine. Not modified.
    * @param secondPolygon the second convex polygon to combine. Not modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time the
    *            other polygons' vertices were edited.
    * @throws ReferenceFrameMismatchException if {@code firstPolygon} and {@code this} are not
    *            expressed in the same reference frame.
    */
   // TODO: Make this more efficient by finding the rotating calipers, as in the intersection method.
   default void setAndUpdate(FrameConvexPolygon2DReadOnly firstPolygon, ConvexPolygon2DReadOnly secondPolygon)
   {
      checkReferenceFrameMatch(firstPolygon);
      ConvexPolygon2DBasics.super.setAndUpdate(firstPolygon, secondPolygon);
   }

   /**
    * Sets this polygon such that it represents the smallest convex hull that contains both polygons.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    *
    * @param firstPolygon the first convex polygon to combine. Not modified.
    * @param secondPolygon the second convex polygon to combine. Not modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time the
    *            other polygons' vertices were edited.
    * @throws ReferenceFrameMismatchException if {@code secondPolygon} and {@code this} are not
    *            expressed in the same reference frame.
    */
   // TODO: Make this more efficient by finding the rotating calipers, as in the intersection method.
   default void setAndUpdate(ConvexPolygon2DReadOnly firstPolygon, FrameConvexPolygon2DReadOnly secondPolygon)
   {
      checkReferenceFrameMatch(secondPolygon);
      ConvexPolygon2DBasics.super.setAndUpdate(firstPolygon, secondPolygon);
   }

   /**
    * Scale this convex polygon about {@code pointToScaleAbout}.
    * <p>
    * This method effectively modifies the vertices of this polygon such that the scale is applied on
    * the distance between each vertex and the given {@code pointToScaleAbout}.
    * </p>
    * <p>
    * If {@code pointToScaleAbout} is equal to a vertex of this polygon, the coordinates of this vertex
    * will remain unmodified.
    * </p>
    *
    * @param pointToScaleAbout the center of the scale transformation. Not modified.
    *
    * @param scaleFactor the scale factor to apply to this polygon. A value of {@code 1.0} does
    *           nothing.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    * @throws ReferenceFrameMismatchException if {@code pointToScaleAbout} and {@code this} are not
    *            expressed in the same reference frame.
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
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    * @throws ReferenceFrameMismatchException if {@code translation} and {@code this} are not expressed
    *            in the same reference frame.
    */
   default void translate(FrameTuple2DReadOnly translation)
   {
      checkReferenceFrameMatch(translation);
      ConvexPolygon2DBasics.super.translate(translation);
   }
}
