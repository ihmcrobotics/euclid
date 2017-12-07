package us.ihmc.euclid.geometry;

import us.ihmc.euclid.geometry.interfaces.Line3DBasics;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Represents an infinitely long 3D line defined by a 3D point and a 3D unitary vector.
 */
public class Line3D implements Line3DBasics, GeometryObject<Line3D>
{
   private final static double minAllowableVectorPart = Math.sqrt(Double.MIN_NORMAL);

   /** Coordinates of a point located on this line. */
   private final Point3D point = new Point3D();
   /** Normalized direction of this line. */
   private final Vector3D direction = new Vector3D();

   /** Whether or not this line's point is set. */
   private boolean pointHasBeenSet = false;
   /** Whether or not this line's direction is set. */
   private boolean directionHasBeenSet = false;

   /**
    * Default constructor that initializes both {@link #point} and {@link #direction} to zero. This
    * point and vector have to be set to valid values to make this line usable.
    */
   public Line3D()
   {
      pointHasBeenSet = false;
      directionHasBeenSet = false;
   }

   /**
    * Initializes this line to be passing through the given point, with the vector as the direction.
    * 
    * @param pointOnLineX the x-coordinate of a point on this line.
    * @param pointOnLineY the y-coordinate of a point on this line.
    * @param pointOnLineZ the z-coordinate of a point on this line.
    * @param lineDirectionX the x-component of the direction of this line.
    * @param lineDirectionY the y-component of the direction of this line.
    * @param lineDirectionZ the z-component of the direction of this line.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   public Line3D(double pointOnLineX, double pointOnLineY, double pointOnLineZ, double lineDirectionX, double lineDirectionY, double lineDirectionZ)
   {
      set(pointOnLineX, pointOnLineY, pointOnLineZ, lineDirectionX, lineDirectionY, lineDirectionZ);
   }

   /**
    * Creates a new line 3D and initializes it to {@code other}.
    * 
    * @param other the other line used to initialize this line. Not modified.
    */
   public Line3D(Line3D other)
   {
      set(other);
   }

   /**
    * Creates a new line 3D and initializes it to {@code other}.
    *
    * @param other the other line used to initialize this line. Not modified.
    */
   public Line3D(Line3DReadOnly other)
   {
      set(other);
   }

   /**
    * Initializes this line to be passing through the two given points.
    * 
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws RuntimeException if the new direction is unreasonably small.
    * @throws RuntimeException if the two given points are exactly equal.
    */
   public Line3D(Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      set(firstPointOnLine, secondPointOnLine);
   }

   /**
    * Initializes this line to be passing through the given point, with the vector as the direction.
    * 
    * @param pointOnLine point on this line. Not modified.
    * @param lineDirection direction of this line. Not modified.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   public Line3D(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      set(pointOnLine, lineDirection);
   }

   /**
    * Transforms this line using the given homogeneous transformation matrix.
    * 
    * @param transform the transform to apply on this line's point and vector. Not modified.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   @Override
   public void applyTransform(Transform transform)
   {
      checkHasBeenInitialized();
      point.applyTransform(transform);
      direction.applyTransform(transform);
   }

   /**
    * Transforms this line using the inverse of the given homogeneous transformation matrix.
    * 
    * @param transform the transform to apply on this line's point and vector. Not modified.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      checkHasBeenInitialized();
      point.applyInverseTransform(transform);
      direction.applyInverseTransform(transform);
   }
   
   /** {@inheritDoc} */
   @Override
   public boolean hasPointBeenSet() {
      return pointHasBeenSet;
   }
   
   /** {@inheritDoc} */
   @Override
   public boolean hasDirectionBeenSet() {
      return directionHasBeenSet;
   }

   /**
    * Tests if this line contains {@link Double#NaN}.
    * 
    * @return {@code true} if {@link #point} and/or {@link #direction} contains {@link Double#NaN},
    *         {@code false} otherwise.
    */
   @Override
   public boolean containsNaN()
   {
      return point.containsNaN() || direction.containsNaN();
   }

   /**
    * Tests on a per-component basis on the point and vector if this line is equal to {@code other}
    * with the tolerance {@code epsilon}. This method will return {@code false} if the two lines are
    * physically the same but either the point or vector of each line is different. For instance, if
    * {@code this.point == other.point} and {@code this.direction == - other.direction}, the two
    * lines are physically the same but this method returns {@code false}.
    * 
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two lines are equal, {@code false} otherwise.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   @Override
   public boolean epsilonEquals(Line3D other, double epsilon)
   {
      checkHasBeenInitialized();
      if (!point.epsilonEquals(other.point, epsilon))
         return false;
      if (!direction.epsilonEquals(other.direction, epsilon))
         return false;

      return true;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(Line3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param obj the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object obj)
   {
      try
      {
         return equals((Line3D) obj);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /** {@inheritDoc} */
   @Override
   public Vector3DReadOnly getDirection()
   {
      checkHasBeenInitialized();
      return direction;
   }

   /** {@inheritDoc} */
   @Override
   public Point3DReadOnly getPoint()
   {
      checkHasBeenInitialized();
      return point;
   }

   /**
    * Flips this line's direction.
    * 
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public void negateDirection()
   {
      checkHasBeenInitialized();
      direction.negate();
   }

   /**
    * Sets this line to be the same as the given line.
    * 
    * @param other the other line to copy. Not modified.
    * @throws RuntimeException if the other line has not been initialized yet.
    */
   @Override
   public void set(Line3D other)
   {
      Line3DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setDirection(double lineDirectionX, double lineDirectionY, double lineDirectionZ)
   {
      setDirectionUnsafe(lineDirectionX, lineDirectionY, lineDirectionZ);
      direction.normalize();
   }

   /** {@inheritDoc} */
   public void setDirectionUnsafe(double lineDirectionX, double lineDirectionY, double lineDirectionZ)
   {
      direction.set(lineDirectionX, lineDirectionY, lineDirectionZ);

      // checkReasonableVector
      if (Math.abs(direction.getX()) < minAllowableVectorPart && Math.abs(direction.getY()) < minAllowableVectorPart
            && Math.abs(direction.getZ()) < minAllowableVectorPart)
      {
         throw new RuntimeException("Line length must be greater than zero");
      }
      
      directionHasBeenSet = true;
   }

   /** {@inheritDoc} */
   @Override
   public void setPoint(double pointOnLineX, double pointOnLineY, double pointOnLineZ)
   {
      this.point.set(pointOnLineX, pointOnLineY, pointOnLineZ);
      pointHasBeenSet = true;
   }

   /**
    * Sets the point and vector of this line to {@link Double#NaN}. After calling this method, this
    * line becomes invalid. A new valid point and valid vector will have to be set so this line is
    * again usable.
    */
   @Override
   public void setToNaN()
   {
      point.setToNaN();
      direction.setToNaN();
   }

   /**
    * Sets the point and vector of this line to zero. After calling this method, this line becomes
    * invalid. A new valid point and valid vector will have to be set so this line is again usable.
    */
   @Override
   public void setToZero()
   {
      point.setToZero();
      direction.setToZero();
      pointHasBeenSet = false;
      directionHasBeenSet = false;
   }

   /**
    * Provides a {@code String} representation of this line 3D as follows:<br>
    * Line 3D: point = (x, y, z), direction = (x, y, z)
    *
    * @return the {@code String} representing this line 3D.
    */
   @Override
   public String toString()
   {
      return EuclidGeometryIOTools.getLine3DString(this);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two lines are geometrically
    * similar.
    * <p>
    * Two lines are considered geometrically equal is they are collinear, pointing toward the same
    * or opposite direction.
    * </p>
    *
    * @param other the line to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two lines represent the same geometry, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(Line3D other, double epsilon)
   {
      return isCollinear(other, epsilon);
   }
}
