package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.geometry.exceptions.EmptyPolygonException;
import us.ihmc.euclid.geometry.exceptions.OutdatedPolygonException;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Write and read interface for a convex polygon defined in the XY-plane.
 * <p>
 * This implementation of convex polygon is designed for garbage free operations.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public interface ConvexPolygon2DBasics extends ConvexPolygon2DReadOnly, Clearable, Transformable
{
   /**
    * Gets the reference to this polygon's axis-aligned bounding box.
    */
   @Override
   BoundingBox2DBasics getBoundingBox();

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
   default void updateBoundingBox()
   {
      BoundingBox2DBasics boundingBox = getBoundingBox();
      boundingBox.setToNaN();
      boundingBox.updateToIncludePoints(this);
   }

   /**
    * Compute centroid and area of this polygon. Formula taken from
    * <a href= "http://local.wasp.uwa.edu.au/~pbourke/geometry/polyarea/">here</a>.
    */
   void updateCentroidAndArea();

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
   Point2DBasics getVertexUnsafe(int index);

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
    * Add a vertex to this polygon using the x and y coordinates of the given {@code vertex}.
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    *
    * @param vertex the new vertex. Not modified.
    */
   default void addVertex(Point3DReadOnly vertex)
   {
      addVertex(vertex.getX(), vertex.getY());
   }

   /**
    * Adds the vertices from the given vertex supplier.
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    *
    * @param vertex2DSupplier the supplier of vertices.
    * @see Vertex2DSupplier
    */
   default void addVertices(Vertex2DSupplier vertex2DSupplier)
   {
      for (int index = 0; index < vertex2DSupplier.getNumberOfVertices(); index++)
      {
         addVertex(vertex2DSupplier.getVertex(index));
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
    * @param vertex3DSupplier the supplier of vertices.
    * @see Vertex3DSupplier
    */
   default void addVertices(Vertex3DSupplier vertex3DSupplier)
   {
      for (int index = 0; index < vertex3DSupplier.getNumberOfVertices(); index++)
      {
         addVertex(vertex3DSupplier.getVertex(index));
      }
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
    * <li>{@link #addVertices(Vertex2DSupplier)}.
    * <li>{@link #update()}.
    * </ol>
    *
    * @param vertex2DSupplier the supplier of vertices.
    * @see Vertex2DSupplier
    * @see #addVertices(Vertex2DSupplier)
    */
   default void set(Vertex2DSupplier vertex2DSupplier)
   {
      clear();
      addVertices(vertex2DSupplier);
      update();
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #clear()}.
    * <li>{@link #addVertices(Vertex3DSupplier)}.
    * <li>{@link #update()}.
    * </ol>
    *
    * @param vertex3DSupplier the supplier of vertices.
    * @see Vertex3DSupplier
    * @see #addVertices(Vertex3DSupplier)
    */
   default void set(Vertex3DSupplier vertex3DSupplier)
   {
      clear();
      addVertices(vertex3DSupplier);
      update();
   }

   /**
    * Sets this polygon such that it represents the smallest convex hull that contains all the vertices
    * supplied by the two suppliers.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    *
    * @param firstVertex2DSupplier the first supplier of vertices.
    * @param secondVertex2DSupplier the second supplier of vertices.
    */
   default void set(Vertex2DSupplier firstVertex2DSupplier, Vertex2DSupplier secondVertex2DSupplier)
   {
      clear();
      addVertices(firstVertex2DSupplier);
      addVertices(secondVertex2DSupplier);
      update();
   }

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
         Point2DBasics vertex = getVertexUnsafe(i);
         vertex.sub(pointToScaleAbout);
         vertex.scale(scaleFactor);
         vertex.add(pointToScaleAbout);
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
         getVertexUnsafe(i).add(x, y);
      }

      updateBoundingBox();
      updateCentroidAndArea();
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
   default void applyTransform(Transform transform)
   {
      applyTransform(transform, true);
   }

   /**
    * Transforms this convex polygon using the given homogeneous transformation matrix.
    *
    * @param transform the transform to apply on the vertices of this convex polygon. Not modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of the
    *           given transform represents a transformation in the XY plane.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the rotation part
    *            of {@code transform} is not a transformation in the XY plane.
    */
   default void applyTransform(Transform transform, boolean checkIfTransformInXYPlane)
   {
      checkIfUpToDate();
      notifyVerticesChanged();

      for (int i = 0; i < getNumberOfVertices(); i++)
      {
         getVertexUnsafe(i).applyTransform(transform, checkIfTransformInXYPlane);
      }
      update();
   }

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
   default void applyInverseTransform(Transform transform)
   {
      applyInverseTransform(transform, true);
   }

   /**
    * Transforms this convex polygon using the inverse of the given homogeneous transformation matrix.
    *
    * @param transform the transform to apply on the vertices of this convex polygon. Not modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of the
    *           given transform represents a transformation in the XY plane.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the rotation part
    *            of {@code transform} is not a transformation in the XY plane.
    */
   default void applyInverseTransform(Transform transform, boolean checkIfTransformInXYPlane)
   {
      checkIfUpToDate();
      notifyVerticesChanged();

      for (int i = 0; i < getNumberOfVertices(); i++)
      {
         getVertexUnsafe(i).applyInverseTransform(transform, checkIfTransformInXYPlane);
      }
      update();
   }
}
