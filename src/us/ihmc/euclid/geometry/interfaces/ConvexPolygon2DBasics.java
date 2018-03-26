package us.ihmc.euclid.geometry.interfaces;

import java.util.List;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.exceptions.EmptyPolygonException;
import us.ihmc.euclid.geometry.exceptions.OutdatedPolygonException;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

public interface ConvexPolygon2DBasics extends ConvexPolygon2DReadOnly, Clearable, Transformable
{
   /**
    * Notifies the vertices have changed and that this polygon has to be updated.
    */
   void notifyVerticesChanged();

   /**
    * After calling this method, the polygon has no vertex, area, or centroid.
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    */
   void clear();

   /**
    * After calling this method, the polygon has no vertex, area, or centroid.
    * <p>
    * Use only when an empty polygon is desired.
    * </p>
    */
   void clearAndUpdate();

   /**
    * Clears this polygon, adds a single vertex at (0, 0), and updates it.
    */
   @Override
   default void setToZero()
   {
      clear();
      addVertex(0.0, 0.0);
      update();
   }

   /**
    * Clears this polygon, adds a single vertex at ({@link Double#NaN}, {@link Double#NaN}), and
    * updates it.
    */
   @Override
   default void setToNaN()
   {
      clear();
      addVertex(Double.NaN, Double.NaN);
      update();
   }

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return ConvexPolygon2DReadOnly.super.containsNaN();
   }

   /**
    * Updates the vertices so they represent a clockwise convex polygon.
    * <p>
    * Call this method after editing the vertices of this polygon.
    * </p>
    * <p>
    * Note that this also updates centroid, area and the bounding box of this polygon.
    * </p>
    */
   void update();

   /**
    * Updates the bounding box properties.
    */
   void updateBoundingBox();

   /**
    * Compute centroid and area of this polygon. Formula taken from
    * <a href= "http://local.wasp.uwa.edu.au/~pbourke/geometry/polyarea/">here</a>.
    */
   void updateCentroidAndArea();

   /**
    * Add a vertex to this polygon.
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    *
    * @param x the x-coordinate of the new vertex.
    * @param y the y-coordinate of the new vertex.
    */
   void addVertex(double x, double y);

   /**
    * Add a vertex to this polygon.
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    *
    * @param vertex the new vertex. Not modified.
    */
   default void addVertex(Point2DReadOnly vertex)
   {
      addVertex(vertex.getX(), vertex.getY());
   }

   /**
    * Adds the N first vertices from the given list to this polygon, where N is equal to
    * {@code numberOfVertices}.
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
    */
   default void addVertices(List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {
      if (numberOfVertices < 0 || numberOfVertices > vertices.size())
         throw new IllegalArgumentException("Illegal numberOfVertices: " + numberOfVertices + ", expected a value in ] 0, " + vertices.size() + "].");

      for (int i = 0; i < numberOfVertices; i++)
         addVertex(vertices.get(i));
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
    */
   default void addVertices(Point2DReadOnly[] vertices, int numberOfVertices)
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
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    *
    * @param vertices the array containing the vertices to add to this polygon. Each row contains one
    *           point whereas the (at least) two columns contains in order the coordinates x and y. Not
    *           modified.
    * @param numberOfVertices specifies the number of relevant points in the array. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the size
    *            of the given array of vertices.
    */
   default void addVertices(double[][] vertices, int numberOfVertices)
   {
      if (numberOfVertices < 0 || numberOfVertices > vertices.length)
         throw new IllegalArgumentException("Illegal numberOfVertices: " + numberOfVertices + ", expected a value in ] 0, " + vertices.length + "].");

      for (int i = 0; i < numberOfVertices; i++)
         addVertex(vertices[i][0], vertices[i][1]);
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
   default void addVertices(ConvexPolygon2DReadOnly otherPolygon)
   {
      for (int i = 0; i < otherPolygon.getNumberOfVertices(); i++)
         addVertex(otherPolygon.getVertex(i));
   }

   /**
    * Removes the vertex of this polygon positioned at the index {@code indexOfVertexToRemove}.
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    *
    * @param indexOfVertexToRemove the index of the vertex to remove.
    * @throws EmptyPolygonException if this polygon is empty before calling this method.
    * @throws IndexOutOfBoundsException if the given index is either negative or greater or equal than
    *            the polygon's number of vertices.
    */
   void removeVertex(int indexOfVertexToRemove);

   /**
    * This method does:
    * <ol>
    * <li>{@link #clear()}.
    * <li>{@link #addVertices(List, int)}.
    * <li>{@link #update()}.
    * </ol>
    *
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the list. Only the points &in;
    *           [0; {@code numberOfVertices}[ are processed.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the size
    *            of the given list of vertices.
    */
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
    * <li>{@link #addVertices(Point2DReadOnly[], int)}.
    * <li>{@link #update()}.
    * </ol>
    *
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the array. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the size
    *            of the given array of vertices.
    */
   default void setAndUpdate(Point2DReadOnly[] vertices, int numberOfVertices)
   {
      clear();
      addVertices(vertices, numberOfVertices);
      update();
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #clear()}.
    * <li>{@link #addVertices(double[][], int)}.
    * <li>{@link #update()}.
    * </ol>
    *
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Each row
    *           contains one point whereas the (at least) two columns contains in order the coordinates
    *           x and y. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the array. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the size
    *            of the given array of vertices.
    */
   default void setAndUpdate(double[][] vertices, int numberOfVertices)
   {
      clear();
      addVertices(vertices, numberOfVertices);
      update();
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #clear()}.
    * <li>{@link #addVertices(ConvexPolygon2DReadOnly)}.
    * <li>{@link #update()}.
    * </ol>
    *
    * @param otherPolygon the other convex polygon to copy. Not modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time the
    *            other polygon's vertices were edited.
    */
   // TODO There is no need to call update() there, instead update everything from the other polygon to make it faster.
   default void setAndUpdate(ConvexPolygon2DReadOnly otherPolygon)
   {
      clear();
      addVertices(otherPolygon);
      update();
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
    */
   // TODO: Make this more efficient by finding the rotating calipers, as in the intersection method.
   default void setAndUpdate(ConvexPolygon2DReadOnly firstPolygon, ConvexPolygon2DReadOnly secondPolygon)
   {
      clear();
      addVertices(firstPolygon);
      addVertices(secondPolygon);
      update();
   }

   void setVertex(int index, double x, double y);

   /**
    * Scale this convex polygon about its centroid.
    * <p>
    * The polygon centroid remains unchanged.
    * </p>
    *
    * @param scaleFactor the scale factor to apply to this polygon. A value of {@code 1.0} does
    *           nothing.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   default void scale(double scaleFactor)
   {
      scale(getCentroid(), scaleFactor);
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
    */
   default void scale(Point2DReadOnly pointToScaleAbout, double scaleFactor)
   {
      checkIfUpToDate();
      notifyVerticesChanged();

      for (int i = 0; i < getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = getUnmodifiableVertexBuffer().get(i);
         double x = pointToScaleAbout.getX() + scaleFactor * (vertex.getX() - pointToScaleAbout.getX());
         double y = pointToScaleAbout.getY() + scaleFactor * (vertex.getY() - pointToScaleAbout.getY());
         setVertex(i, x, y);
      }

      update();
   }

   /**
    * Translates this polygon.
    *
    * @param translation the translation to apply to this polygon's vertices. Not modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   default void translate(Tuple2DReadOnly translation)
   {
      translate(translation.getX(), translation.getY());
   }

   /**
    * Translated this polygon.
    *
    * @param x the translation along the x-axis to apply to each of this polygon's vertices.
    * @param y the translation along the y-axis to apply to each of this polygon's vertices.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   default void translate(double x, double y)
   {
      checkIfUpToDate();

      for (int i = 0; i < getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = getUnmodifiableVertexBuffer().get(i);
         setVertex(i, vertex.getX() + x, vertex.getY() + y);
      }

      updateBoundingBox();
      updateCentroidAndArea();
   }

   /**
    * Copies this polygon, translates the copy, and returns it.
    *
    * @param translation the translation to apply to the copy of this polygon. Not modified.
    * @return the copy of this polygon translated.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   default ConvexPolygon2DBasics translateCopy(Tuple2DReadOnly translation)
   {
      ConvexPolygon2D copy = new ConvexPolygon2D(this);
      copy.translate(translation);
      return copy;
   }

   /**
    * Transforms this convex polygon using the given homogeneous transformation matrix.
    *
    * @param transform the transform to apply on the vertices of this convex polygon. Not modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a transformation
    *            in the XY-plane.
    */
   @Override
   void applyTransform(Transform transform);

   /**
    * Transforms this convex polygon using the inverse of the given homogeneous transformation matrix.
    *
    * @param transform the transform to apply on the vertices of this convex polygon. Not modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a transformation
    *            in the XY-plane.
    */
   @Override
   void applyInverseTransform(Transform transform);

   /**
    * Transforms this convex polygon using the given homogeneous transformation matrix and project the
    * result onto the XY-plane.
    *
    * @param transform the transform to apply on the vertices of this convex polygon. Not modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   void applyTransformAndProjectToXYPlane(Transform transform);
}
