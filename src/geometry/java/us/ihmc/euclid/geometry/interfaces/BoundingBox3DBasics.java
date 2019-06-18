package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Write and read interface for a 3D axis-aligned bounding box defined from a set of minimum and
 * maximum coordinates.
 */
public interface BoundingBox3DBasics extends BoundingBox3DReadOnly, Clearable
{
   /**
    * Gets the reference to the minimum coordinate of this bounding box.
    *
    * @return the reference to the minimum coordinate.
    */
   @Override
   Point3DBasics getMinPoint();

   /**
    * Gets the reference to the maximum coordinate of this bounding box.
    *
    * @return the reference to the maximum coordinate.
    */
   @Override
   Point3DBasics getMaxPoint();

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return BoundingBox3DReadOnly.super.containsNaN();
   }

   /**
    * Sets the minimum coordinate of this bounding box.
    *
    * @param min the minimum coordinate for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *                          coordinate on the same axis.
    */
   default void setMin(Point3DReadOnly min)
   {
      getMinPoint().set(min);
      checkBounds();
   }

   /**
    * Sets the minimum coordinate of this bounding box.
    *
    * @param min the minimum coordinates for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *                          coordinate on the same axis.
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
    * @param minZ the new minimum z-coordinate for this bounding box.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *                          coordinate on the same axis.
    */
   default void setMin(double minX, double minY, double minZ)
   {
      getMinPoint().set(minX, minY, minZ);
      checkBounds();
   }

   /**
    * Sets the maximum coordinate of this bounding box.
    *
    * @param max the maximum coordinate for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *                          coordinate on the same axis.
    */
   default void setMax(Point3DReadOnly max)
   {
      getMaxPoint().set(max);
      checkBounds();
   }

   /**
    * Sets the maximum coordinate of this bounding box.
    *
    * @param max the maximum coordinates for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *                          coordinate on the same axis.
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
    * @param maxZ the new maximum z-coordinate for this bounding box.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *                          coordinate on the same axis.
    */
   default void setMax(double maxX, double maxY, double maxZ)
   {
      getMaxPoint().set(maxX, maxY, maxZ);
      checkBounds();
   }

   /**
    * Redefines this bounding box with new minimum and maximum coordinates.
    *
    * @param minX the new minimum x-coordinate for this bounding box.
    * @param minY the new minimum y-coordinate for this bounding box.
    * @param minZ the new minimum z-coordinate for this bounding box.
    * @param maxX the new maximum x-coordinates for this bounding box.
    * @param maxY the new maximum y-coordinates for this bounding box.
    * @param maxZ the new maximum z-coordinates for this bounding box.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *                          coordinate on the same axis.
    */
   default void set(double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
   {
      getMinPoint().set(minX, minY, minZ);
      getMaxPoint().set(maxX, maxY, maxZ);
      checkBounds();
   }

   /**
    * Redefines this bounding box with new minimum and maximum coordinates.
    *
    * @param min the new minimum coordinates for this bounding box. Not modified.
    * @param max the new maximum coordinates for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *                          coordinate on the same axis.
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
    *                          coordinate on the same axis.
    */
   default void set(Point3DReadOnly min, Point3DReadOnly max)
   {
      getMinPoint().set(min);
      getMaxPoint().set(max);
      checkBounds();
   }

   /**
    * Redefines this bounding box given its {@code center} location and half its size along each axis
    * {@code halfSize}.
    *
    * @param center   the new center location of this bounding box. Not modified.
    * @param halfSize half the size of this bounding box. Not modified.
    */
   default void set(Point3DReadOnly center, Vector3DReadOnly halfSize)
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
   default void set(BoundingBox3DReadOnly other)
   {
      getMinPoint().set(other.getMinPoint());
      getMaxPoint().set(other.getMaxPoint());
   }

   /**
    * Combines this bounding box with {@code other} such that it becomes the smallest bounding box
    * containing this and {@code other}.
    * <p>
    * For each coordinates, if it is {@link Double#NaN} for one of the two bounding boxes, then the
    * coordinate of the other bounding box is used to update this bounding box. As a result, if for
    * instance {@code this} was set to {@link Double#NaN} beforehand, this operation would be
    * equivalent to {@code this.set(other}.
    * </p>
    *
    * @param other the other bounding box to combine with this. Not modified.
    */
   default void combine(BoundingBox3DReadOnly other)
   {
      combine(this, other);
   }

   /**
    * Sets this bounding box to the union of {@code boundingBoxOne} and {@code boundingBoxTwo}.
    * <p>
    * This bounding box is set such that it is the smallest bounding box containing the two given
    * bounding boxes.
    * </p>
    * <p>
    * For each coordinates, if it is {@link Double#NaN} for one of the two bounding boxes, then the
    * coordinate of the other bounding box is used to update this bounding box. As a result, if for
    * instance {@code boundingBoxOne} was set to {@link Double#NaN} beforehand, this operation would be
    * equivalent to {@code this.set(boundingBoxTwo}.
    * </p>
    *
    * @param boundingBoxOne the first bounding box. Can be the same instance as this. Not modified.
    * @param boundingBoxTwo the second bounding box. Can be the same instance as this. Not modified.
    */
   default void combine(BoundingBox3DReadOnly boundingBoxOne, BoundingBox3DReadOnly boundingBoxTwo)
   {
      double minX, minY, minZ;

      if (Double.isNaN(boundingBoxOne.getMinX()))
         minX = boundingBoxTwo.getMinX();
      else if (Double.isNaN(boundingBoxTwo.getMinX()))
         minX = boundingBoxOne.getMinX();
      else
         minX = Math.min(boundingBoxOne.getMinX(), boundingBoxTwo.getMinX());

      if (Double.isNaN(boundingBoxOne.getMinY()))
         minY = boundingBoxTwo.getMinY();
      else if (Double.isNaN(boundingBoxTwo.getMinY()))
         minY = boundingBoxOne.getMinY();
      else
         minY = Math.min(boundingBoxOne.getMinY(), boundingBoxTwo.getMinY());

      if (Double.isNaN(boundingBoxOne.getMinZ()))
         minZ = boundingBoxTwo.getMinZ();
      else if (Double.isNaN(boundingBoxTwo.getMinZ()))
         minZ = boundingBoxOne.getMinZ();
      else
         minZ = Math.min(boundingBoxOne.getMinZ(), boundingBoxTwo.getMinZ());

      double maxX, maxY, maxZ;

      if (Double.isNaN(boundingBoxOne.getMaxX()))
         maxX = boundingBoxTwo.getMaxX();
      else if (Double.isNaN(boundingBoxTwo.getMaxX()))
         maxX = boundingBoxOne.getMaxX();
      else
         maxX = Math.max(boundingBoxOne.getMaxX(), boundingBoxTwo.getMaxX());

      if (Double.isNaN(boundingBoxOne.getMaxY()))
         maxY = boundingBoxTwo.getMaxY();
      else if (Double.isNaN(boundingBoxTwo.getMaxY()))
         maxY = boundingBoxOne.getMaxY();
      else
         maxY = Math.max(boundingBoxOne.getMaxY(), boundingBoxTwo.getMaxY());

      if (Double.isNaN(boundingBoxOne.getMaxZ()))
         maxZ = boundingBoxTwo.getMaxZ();
      else if (Double.isNaN(boundingBoxTwo.getMaxZ()))
         maxZ = boundingBoxOne.getMaxZ();
      else
         maxZ = Math.max(boundingBoxOne.getMaxZ(), boundingBoxTwo.getMaxZ());

      set(minX, minY, minZ, maxX, maxY, maxZ);
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
    * Updates this bounding box to be the smallest bounding box that includes this and the given point.
    *
    * @param point the point to be included in this bounding box. Not modified.
    */
   default void updateToIncludePoint(Point3DReadOnly point)
   {
      this.updateToIncludePoint(point.getX(), point.getY(), point.getZ());
   }

   /**
    * Updates this bounding box to be the smallest bounding box that includes this and the given point.
    *
    * @param x x-coordinate of the point to be included in this bounding box. Not modified.
    * @param y y-coordinate of the point to be included in this bounding box. Not modified.
    * @param z z-coordinate of the point to be included in this bounding box. Not modified.
    */
   default void updateToIncludePoint(double x, double y, double z)
   {
      if (Double.isNaN(getMinX()) || x < getMinX())
      {
         getMinPoint().setX(x);
      }

      if (Double.isNaN(getMinY()) || y < getMinY())
      {
         getMinPoint().setY(y);
      }

      if (Double.isNaN(getMinZ()) || z < getMinZ())
      {
         getMinPoint().setZ(z);
      }

      if (Double.isNaN(getMaxX()) || x > getMaxX())
      {
         getMaxPoint().setX(x);
      }

      if (Double.isNaN(getMaxY()) || y > getMaxY())
      {
         getMaxPoint().setY(y);
      }

      if (Double.isNaN(getMaxZ()) || z > getMaxZ())
      {
         getMaxPoint().setZ(z);
      }
   }
}
