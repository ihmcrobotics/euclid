package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

/**
 * Write and read interface for a 2D axis-aligned bounding box defined from a set of minimum and
 * maximum coordinates.
 */
public interface BoundingBox2DBasics extends BoundingBox2DReadOnly, Clearable
{
   /**
    * Gets the reference to the minimum coordinate of this bounding box.
    *
    * @return the reference to the minimum coordinate.
    */
   @Override
   Point2DBasics getMinPoint();

   /**
    * Gets the reference to the maximum coordinate of this bounding box.
    *
    * @return the reference to the maximum coordinate.
    */
   @Override
   Point2DBasics getMaxPoint();

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return BoundingBox2DReadOnly.super.containsNaN();
   }

   /**
    * Sets the minimum coordinate of this bounding box.
    *
    * @param min the minimum coordinate for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   default void setMin(Point2DReadOnly min)
   {
      getMinPoint().set(min);
      checkBounds();
   }

   /**
    * Sets the minimum coordinate of this bounding box.
    *
    * @param min the minimum coordinates for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   default void setMin(double[] min)
   {
      getMinPoint().set(min);
      checkBounds();
   }

   /**
    * Sets the minimum coordinate of this bounding box.
    *
    * @param minX the new minimum x-coordinate for this bounding box.
    * @param minY the new minimum y-coordinate for this bounding box.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   default void setMin(double minX, double minY)
   {
      getMinPoint().set(minX, minY);
      checkBounds();
   }

   /**
    * Sets the maximum coordinate of this bounding box.
    *
    * @param max the maximum coordinate for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   default void setMax(Point2DReadOnly max)
   {
      getMaxPoint().set(max);
      checkBounds();
   }

   /**
    * Sets the maximum coordinate of this bounding box.
    *
    * @param max the maximum coordinates for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   default void setMax(double[] max)
   {
      getMaxPoint().set(max);
      checkBounds();
   }

   /**
    * Sets the maximum coordinate of this bounding box.
    *
    * @param maxX the new maximum x-coordinate for this bounding box.
    * @param maxY the new maximum y-coordinate for this bounding box.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   default void setMax(double maxX, double maxY)
   {
      getMaxPoint().set(maxX, maxY);
      checkBounds();
   }

   /**
    * Redefines this bounding box with new minimum and maximum coordinates.
    *
    * @param minX the new minimum x-coordinate for this bounding box.
    * @param minY the new minimum y-coordinate for this bounding box.
    * @param maxX the new maximum x-coordinates for this bounding box.
    * @param maxY the new maximum y-coordinates for this bounding box.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   default void set(double minX, double minY, double maxX, double maxY)
   {
      getMinPoint().set(minX, minY);
      getMaxPoint().set(maxX, maxY);
      checkBounds();
   }

   /**
    * Redefines this bounding box with new minimum and maximum coordinates.
    *
    * @param min the new minimum coordinates for this bounding box. Not modified.
    * @param max the new maximum coordinates for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   default void set(double[] min, double[] max)
   {
      getMinPoint().set(min);
      getMaxPoint().set(max);
      checkBounds();
   }

   /**
    * Redefines this bounding box with new minimum and maximum coordinates.
    *
    * @param min the new minimum coordinates for this bounding box. Not modified.
    * @param max the new maximum coordinates for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   default void set(Point2DReadOnly min, Point2DReadOnly max)
   {
      getMinPoint().set(min);
      getMaxPoint().set(max);
      checkBounds();
   }

   /**
    * Redefines this bounding box given its {@code center} location and half its size along each axis
    * {@code halfSize}.
    *
    * @param center the new center location of this bounding box. Not modified.
    * @param halfSize half the size of this bounding box. Not modified.
    */
   default void set(Point2DReadOnly center, Vector2DReadOnly halfSize)
   {
      getMinPoint().sub(center, halfSize);
      getMaxPoint().add(center, halfSize);
      checkBounds();
   }

   /**
    * Redefines this bounding box to be the same as the given {@code other}.
    *
    * @param other the bounding box used to redefine this bounding box. Not modified.
    */
   default void set(BoundingBox2DReadOnly other)
   {
      getMinPoint().set(other.getMinPoint());
      getMaxPoint().set(other.getMaxPoint());
   }

   /**
    * Combines this bounding box with {@code other} such that it becomes the smallest bounding box
    * containing this and {@code other}.
    *
    * @param other the other bounding box to combine with this. Not modified.
    */
   default void combine(BoundingBox2DReadOnly other)
   {
      combine(this, other);
   }

   /**
    * Sets this bounding box to the union of {@code boundingBoxOne} and {@code boundingBoxTwo}.
    * <p>
    * This bounding box is set such that it is the smallest bounding box containing the two given
    * bounding boxes.
    * </p>
    *
    * @param boundingBoxOne the first bounding box. Can be the same instance as this. Not modified.
    * @param boundingBoxTwo the second bounding box. Can be the same instance as this. Not modified.
    */
   default void combine(BoundingBox2DReadOnly boundingBoxOne, BoundingBox2DReadOnly boundingBoxTwo)
   {
      double minX = Math.min(boundingBoxOne.getMinX(), boundingBoxTwo.getMinX());
      double minY = Math.min(boundingBoxOne.getMinY(), boundingBoxTwo.getMinY());

      double maxX = Math.max(boundingBoxOne.getMaxX(), boundingBoxTwo.getMaxX());
      double maxY = Math.max(boundingBoxOne.getMaxY(), boundingBoxTwo.getMaxY());

      set(minX, minY, maxX, maxY);
   }

   /**
    * Invalidates this bounding box by setting all its coordinates to {@link Double#NaN}.
    */
   @Override
   default void setToNaN()
   {
      getMinPoint().setToNaN();
      getMaxPoint().setToNaN();
   }

   /**
    * Resets this bounding box by setting both its minimum and maximum coordinates to (0, 0).
    */
   @Override
   default void setToZero()
   {
      getMinPoint().setToZero();
      getMaxPoint().setToZero();
   }

   /**
    * Updates this bounding box to be the smallest bounding box that includes this and the supplied
    * points.
    *
    * @param vertex2DSupplier the supply of points.
    */
   default void updateToIncludePoints(Vertex2DSupplier vertex2DSupplier)
   {
      for (int index = 0; index < vertex2DSupplier.getNumberOfVertices(); index++)
         updateToIncludePoint(vertex2DSupplier.getVertex(index));
   }

   /**
    * Updates this bounding box to be the smallest bounding box that includes this and the given point.
    *
    * @param point the point to be included in this bounding box. Not modified.
    */
   default void updateToIncludePoint(Point2DReadOnly point)
   {
      updateToIncludePoint(point.getX(), point.getY());
   }

   /**
    * Updates this bounding box to be the smallest bounding box that includes this and the given point.
    *
    * @param x x-coordinate of the point to be included in this bounding box. Not modified.
    * @param y y-coordinate of the point to be included in this bounding box. Not modified.
    */
   default void updateToIncludePoint(double x, double y)
   {
      if (Double.isNaN(getMinPoint().getX()) || x < getMinPoint().getX())
      {
         getMinPoint().setX(x);
      }

      if (Double.isNaN(getMinPoint().getY()) || y < getMinPoint().getY())
      {
         getMinPoint().setY(y);
      }

      if (Double.isNaN(getMaxPoint().getX()) || x > getMaxPoint().getX())
      {
         getMaxPoint().setX(x);
      }

      if (Double.isNaN(getMaxPoint().getY()) || y > getMaxPoint().getY())
      {
         getMaxPoint().setY(y);
      }
   }
}
