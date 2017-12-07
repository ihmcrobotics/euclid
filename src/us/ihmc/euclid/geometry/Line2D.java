package us.ihmc.euclid.geometry;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.geometry.exceptions.OutdatedPolygonException;
import us.ihmc.euclid.geometry.interfaces.Line2DBasics;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

/**
 * Represents an infinitely-long 2D line defined by a 2D point and a 2D unit-vector.
 */
public class Line2D implements Line2DBasics, GeometryObject<Line2D>
{
   private final static double minAllowableVectorPart = Math.sqrt(Double.MIN_NORMAL);

   /** Coordinates of a point located on this line. */
   private final Point2D point = new Point2D();
   /** Normalized direction of this line. */
   private final Vector2D direction = new Vector2D();

   /** Whether or not this line's point is set. */
   private boolean pointHasBeenSet = false;
   /** Whether or not this line's direction is set. */
   private boolean directionHasBeenSet = false;

   /**
    * Default constructor that initializes both {@link #point} and {@link #direction} to zero. This
    * point and vector have to be set to valid values to make this line usable.
    */
   public Line2D()
   {
      pointHasBeenSet = false;
      directionHasBeenSet = false;
   }

   /**
    * Initializes this line to be passing through the given point, with the vector as the direction.
    * 
    * @param pointOnLineX the x-coordinate of a point on this line.
    * @param pointOnLineY the y-coordinate of a point on this line.
    * @param lineDirectionX the x-component of the direction of this line.
    * @param lineDirectionY the y-component of the direction of this line.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   public Line2D(double pointOnLineX, double pointOnLineY, double lineDirectionX, double lineDirectionY)
   {
      set(pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY);
   }

   /**
    * Creates a new line 2D and initializes it to {@code other}.
    * 
    * @param other the other line used to initialize this line. Not modified.
    * @throws RuntimeException if the other line has not been initialized yet.
    */
   public Line2D(Line2D other)
   {
      set(other);
   }

   /**
    * Creates a new line 2D and initializes it to {@code other}.
    *
    * @param other the other line used to initialize this line. Not modified.
    * @throws RuntimeException if the other line has not been initialized yet.
    */
   public Line2D(Line2DReadOnly other)
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
   public Line2D(Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
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
   public Line2D(Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      set(pointOnLine, lineDirection);
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
    * Transforms this line using the given homogeneous transformation matrix.
    * 
    * @param transform the transform to apply on this line's point and vector. Not modified.
    * @throws RuntimeException if this line has not been initialized yet.
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a
    *            transformation in the XY-plane.
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
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a
    *            transformation in the XY-plane.
    */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      checkHasBeenInitialized();
      point.applyInverseTransform(transform);
      direction.applyInverseTransform(transform);
   }

   /**
    * Transforms this line using the given homogeneous transformation matrix and project the result
    * onto the XY-plane.
    * 
    * @param transform the transform to apply on this line's point and vector. Not modified.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   public void applyTransformAndProjectToXYPlane(Transform transform)
   {
      checkHasBeenInitialized();
      point.applyTransform(transform, false);
      direction.applyTransform(transform, false);
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
   public boolean epsilonEquals(Line2D other, double epsilon)
   {
      checkHasBeenInitialized();
      if (!point.epsilonEquals(other.point, epsilon))
         return false;
      if (!direction.epsilonEquals(other.direction, epsilon))
         return false;

      return true;
   }

   /**
    * Tests on a per component basis, if this line 2D is exactly equal to {@code other}.
    *
    * @param other the other line 2D to compare against this. Not modified.
    * @return {@code true} if the two lines are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   public boolean equals(Line2D other)
   {
      if (other == null)
         return false;
      else
         return point.equals(other.point) && direction.equals(other.direction);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(Line2D)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object obj)
   {
      try
      {
         return equals((Line2D) obj);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Gets the read-only reference to the direction of this line.
    * 
    * @return the reference to the direction.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   @Override
   public Vector2DReadOnly getDirection()
   {
      checkHasBeenInitialized();
      return direction;
   }

   /**
    * Gets the read-only reference to the point through which this line is going.
    * 
    * @return the reference to the point.
    * @throws RuntimeException if this line has not been initialized yet.
    */
   @Override
   public Point2DReadOnly getPoint()
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
   public void set(Line2D other)
   {
      Line2DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setDirection(double lineDirectionX, double lineDirectionY)
   {
      setDirectionUnsafe(lineDirectionX, lineDirectionY);      
      direction.normalize();
   }

   /** {@inheritDoc} */
   @Override
   public void setDirectionUnsafe(double lineDirectionX, double lineDirectionY)
   {
      direction.set(lineDirectionX, lineDirectionY);

      // checkReasonableVector
      if (Math.abs(direction.getX()) < minAllowableVectorPart && Math.abs(direction.getY()) < minAllowableVectorPart)
      {
         throw new RuntimeException("Line length must be greater than zero.");
      }
      
      directionHasBeenSet = true;
   }

   /** {@inheritDoc} */
   @Override
   public void setPoint(double pointOnLineX, double pointOnLineY)
   {
      point.set(pointOnLineX, pointOnLineY);
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
      pointHasBeenSet = false;
      directionHasBeenSet = false;
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
    * Provides a {@code String} representation of this line 2D as follows:<br>
    * Line 2D: point = (x, y), direction = (x, y)
    *
    * @return the {@code String} representing this line 2D.
    */
   @Override
   public String toString()
   {
      return EuclidGeometryIOTools.getLine2DString(this);
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
   public boolean geometricallyEquals(Line2D other, double epsilon)
   {
      return isCollinear(other, epsilon);
   }
}
