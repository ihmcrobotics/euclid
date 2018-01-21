package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface Line3DBasics extends Line3DReadOnly, Transformable, Clearable
{
   /**
    * Gets the reference to the point through which this line is going.
    *
    * @return the reference to the point.
    */
   @Override
   Point3DBasics getPoint();

   /**
    * Gets the reference to the direction of this line.
    *
    * @return the reference to the direction.
    */
   @Override
   Vector3DBasics getDirection();

   /**
    * Tests if this line contains {@link Double#NaN}.
    * 
    * @return {@code true} if {@link #point} and/or {@link #direction} contains {@link Double#NaN},
    *         {@code false} otherwise.
    */
   @Override
   default boolean containsNaN()
   {
      return getPoint().containsNaN() || getDirection().containsNaN();
   }

   /**
    * Sets the point and vector of this line to zero. After calling this method, this line becomes
    * invalid. A new valid point and valid vector will have to be set so this line is again usable.
    */
   @Override
   default void setToZero()
   {
      getPoint().setToZero();
      getDirection().setToZero();
   }

   /**
    * Sets the point and vector of this line to {@link Double#NaN}. After calling this method, this
    * line becomes invalid. A new valid point and valid vector will have to be set so this line is
    * again usable.
    */
   @Override
   default void setToNaN()
   {
      getPoint().setToNaN();
      getDirection().setToNaN();
   }

   /**
    * Flips this line's direction.
    */
   default void negateDirection()
   {
      getDirection().negate();
   }

   /**
    * Changes the point through which this line has to go.
    *
    * @param pointOnLineX the new x-coordinate of the point on this line.
    * @param pointOnLineY the new y-coordinate of the point on this line.
    * @param pointOnLineZ the new z-coordinate of the point on this line.
    */
   default void setPoint(double pointOnLineX, double pointOnLineY, double pointOnLineZ)
   {
      getPoint().set(pointOnLineX, pointOnLineY, pointOnLineZ);
   }

   /**
    * Changes the point through which this line has to go.
    *
    * @param pointOnLine new point on this line. Not modified.
    */
   default void setPoint(Point3DReadOnly pointOnLine)
   {
      getPoint().set(pointOnLine);
   }

   /**
    * Changes the direction of this line by setting it to the normalized values provided.
    *
    * @param lineDirectionX the new x-component of the direction of this line.
    * @param lineDirectionY the new y-component of the direction of this line.
    * @param lineDirectionZ the new z-component of the direction of this line.
    */
   default void setDirection(double lineDirectionX, double lineDirectionY, double lineDirectionZ)
   {
      getDirection().set(lineDirectionX, lineDirectionY, lineDirectionZ);
      getDirection().normalize();
   }

   /**
    * Changes the direction of this line by setting it to the normalized value of the given vector.
    *
    * @param lineDirection new direction of this line. Not modified.
    */
   default void setDirection(Vector3DReadOnly lineDirection)
   {
      getDirection().set(lineDirection);
      getDirection().normalize();
   }

   default void set(Line3DReadOnly other)
   {
      getPoint().set(other.getPoint());
      getDirection().set(other.getDirection());
   }

   default void set(LineSegment3DReadOnly lineSegment3DReadOnly)
   {
      set(lineSegment3DReadOnly.getFirstEndpoint(), lineSegment3DReadOnly.getSecondEndpoint());
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    *
    * @param pointOnLineX the new x-coordinate of the point on this line.
    * @param pointOnLineY the new y-coordinate of the point on this line.
    * @param pointOnLineZ the new z-coordinate of the point on this line.
    * @param lineDirectionX the new x-component of the direction of this line.
    * @param lineDirectionY the new y-component of the direction of this line.
    * @param lineDirectionZ the new z-component of the direction of this line.
    */
   default void set(double pointOnLineX, double pointOnLineY, double pointOnLineZ, double lineDirectionX, double lineDirectionY, double lineDirectionZ)
   {
      setPoint(pointOnLineX, pointOnLineY, pointOnLineZ);
      setDirection(lineDirectionX, lineDirectionY, lineDirectionZ);
   }

   /**
    * Redefines this line such that it goes through the two given points.
    *
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws RuntimeException if the two given points are exactly equal.
    */
   default void set(Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      // checkDistinctPoints
      if (firstPointOnLine.equals(secondPointOnLine))
      {
         throw new RuntimeException("Tried to create a line from two coincidal points");
      }

      setPoint(firstPointOnLine);
      getDirection().sub(secondPointOnLine, firstPointOnLine);
      getDirection().normalize();
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    *
    * @param pointOnLine new point on this line. Not modified.
    * @param lineDirection new direction of this line. Not modified.
    */
   default void set(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      setPoint(pointOnLine);
      setDirection(lineDirection);
   }

   /**
    * Translates this line by the given (x, y, z).
    * <p>
    * Note that this line's direction remains unchanged.
    * </p>
    *
    * @param x the distance to translate this line along the x-axis.
    * @param y the distance to translate this line along the y-axis.
    * @param z the distance to translate this line along the z-axis.
    */
   default void translate(double x, double y, double z)
   {
      getPoint().add(x, y, z);
   }

   /**
    * Transforms this line using the given homogeneous transformation matrix.
    * 
    * @param transform the transform to apply on this line's point and vector. Not modified.
    */
   @Override
   default void applyTransform(Transform transform)
   {
      getPoint().applyTransform(transform);
      getDirection().applyTransform(transform);
   }

   /**
    * Transforms this line using the inverse of the given homogeneous transformation matrix.
    * 
    * @param transform the transform to apply on this line's point and vector. Not modified.
    */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      getPoint().applyInverseTransform(transform);
      getDirection().applyInverseTransform(transform);
   }
}
