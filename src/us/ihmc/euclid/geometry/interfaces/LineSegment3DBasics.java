package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Write and read interface for a line segment 3D.
 * <p>
 * A line segment 3D is a finite-length line defined in the XY-plane by its two 3D endpoints.
 * </p>
 */
public interface LineSegment3DBasics extends LineSegment3DReadOnly, Clearable, Transformable
{
   /**
    * Gets the reference to the first endpoint of this line segment.
    *
    * @return the reference to the first endpoint of this line segment.
    */
   @Override
   Point3DBasics getFirstEndpoint();

   /**
    * Gets the reference to the second endpoint of this line segment.
    *
    * @return the reference to the second endpoint of this line segment.
    */
   @Override
   Point3DBasics getSecondEndpoint();

   /**
    * Tests if this line segment contains {@link Double#NaN}.
    *
    * @return {@code true} if {@link #getFirstEndpoint()} and/or {@link #getSecondEndpoint()} contains
    *         {@link Double#NaN}, {@code false} otherwise.
    */
   @Override
   default boolean containsNaN()
   {
      return getFirstEndpoint().containsNaN() || getSecondEndpoint().containsNaN();
   }

   /**
    * Sets both endpoints of this line segment to zero.
    */
   @Override
   default void setToZero()
   {
      getFirstEndpoint().setToZero();
      getSecondEndpoint().setToZero();
   }

   /**
    * Sets both endpoints of this line segment to {@link Double#NaN}. After calling this method, this
    * line segment becomes invalid. A new pair of valid endpoints will have to be set so this line
    * segment is again usable.
    */
   @Override
   default void setToNaN()
   {
      getFirstEndpoint().setToNaN();
      getSecondEndpoint().setToNaN();
   }

   /**
    * Changes the first endpoint of this line segment.
    *
    * @param firstEndpointX x-coordinate of the new first endpoint.
    * @param firstEndpointY y-coordinate of the new first endpoint.
    * @param firstEndpointZ z-coordinate of the new first endpoint.
    */
   default void setFirstEndpoint(double firstEndpointX, double firstEndpointY, double firstEndpointZ)
   {
      getFirstEndpoint().set(firstEndpointX, firstEndpointY, firstEndpointZ);
   }

   /**
    * Changes the second endpoint of this line segment.
    *
    * @param secondEndpointX x-coordinate of the new second endpoint.
    * @param secondEndpointY y-coordinate of the new second endpoint.
    * @param secondEndpointZ z-coordinate of the new second endpoint.
    */
   default void setSecondEndpoint(double secondEndpointX, double secondEndpointY, double secondEndpointZ)
   {
      getSecondEndpoint().set(secondEndpointX, secondEndpointY, secondEndpointZ);
   }

   /**
    * Changes the first endpoint of this line segment.
    *
    * @param firstEndpoint new endpoint of this line segment. Not modified.
    */
   default void setFirstEndpoint(Point3DReadOnly firstEndpoint)
   {
      getFirstEndpoint().set(firstEndpoint);
   }

   /**
    * Changes the second endpoint of this line segment.
    *
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    */
   default void setSecondEndpoint(Point3DReadOnly secondEndpoint)
   {
      getSecondEndpoint().set(secondEndpoint);
   }

   /**
    * Sets this line segment to be same as the given line segment.
    *
    * @param other the other line segment to copy. Not modified.
    */
   default void set(LineSegment3DReadOnly other)
   {
      set(other.getFirstEndpoint(), other.getSecondEndpoint());
   }

   /**
    * Redefines this line segments with new endpoints.
    *
    * @param firstEndpointX x-coordinate of the new first endpoint.
    * @param firstEndpointY y-coordinate of the new first endpoint.
    * @param firstEndpointZ z-coordinate of the new first endpoint.
    * @param secondEndpointX x-coordinate of the new second endpoint.
    * @param secondEndpointY y-coordinate of the new second endpoint.
    * @param secondEndpointZ z-coordinate of the new second endpoint.
    */
   default void set(double firstEndpointX, double firstEndpointY, double firstEndpointZ, double secondEndpointX, double secondEndpointY, double secondEndpointZ)
   {
      setFirstEndpoint(firstEndpointX, firstEndpointY, firstEndpointZ);
      setSecondEndpoint(secondEndpointX, secondEndpointY, secondEndpointZ);
   }

   /**
    * Redefines this line segment with new endpoints.
    *
    * @param firstEndpoint new endpoint of this line segment. Not modified
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    */
   default void set(Point3DReadOnly firstEndpoint, Point3DReadOnly secondEndpoint)
   {
      setFirstEndpoint(firstEndpoint);
      setSecondEndpoint(secondEndpoint);
   }

   /**
    * Redefines this line segment with a new first endpoint and a vector going from the first to the
    * second endpoint.
    *
    * @param firstEndpoint new first endpoint. Not modified.
    * @param fromFirstToSecondEndpoint vector going from the first to the second endpoint. Not
    *           modified.
    */
   default void set(Point3DReadOnly firstEndpoint, Vector3DReadOnly fromFirstToSecondEndpoint)
   {
      getFirstEndpoint().set(firstEndpoint);
      getSecondEndpoint().add(firstEndpoint, fromFirstToSecondEndpoint);
   }

   /**
    * Swaps this line segment's endpoints.
    */
   default void flipDirection()
   {
      double x = getFirstEndpointX();
      double y = getFirstEndpointY();
      double z = getFirstEndpointZ();

      setFirstEndpoint(getSecondEndpoint());
      setSecondEndpoint(x, y, z);
   }

   /**
    * Translates this line segment by the given (x, y, z).
    * <p>
    * Note that the length and direction of this line segment remains unchanged.
    * </p>
    *
    * @param x the distance to translate this line along the x-axis.
    * @param y the distance to translate this line along the y-axis.
    * @param z the distance to translate this line along the z-axis.
    */
   default void translate(double x, double y, double z)
   {
      getFirstEndpoint().add(x, y, z);
      getSecondEndpoint().add(x, y, z);
   }

   /**
    * Translates this line segment by the given (x, y, z) contained in {@code translation}.
    * <p>
    * Note that the length and direction of this line segment remains unchanged.
    * </p>
    *
    * @param translation the translation to add to each endpoint of this line segment. Not modified.
    */
   default void translate(Tuple3DReadOnly translation)
   {
      translate(translation.getX(), translation.getY(), translation.getZ());
   }

   /**
    * Transforms this line segment using the given homogeneous transformation matrix.
    *
    * @param transform the transform to apply on the endpoints of this line segment. Not modified.
    */
   @Override
   default void applyTransform(Transform transform)
   {
      getFirstEndpoint().applyTransform(transform);
      getSecondEndpoint().applyTransform(transform);
   }

   /**
    * Transforms this line segment using the inverse of the given homogeneous transformation matrix.
    *
    * @param transform the transform to apply on the endpoints of this line segment. Not modified.
    */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      getFirstEndpoint().applyInverseTransform(transform);
      getSecondEndpoint().applyInverseTransform(transform);
   }
}
