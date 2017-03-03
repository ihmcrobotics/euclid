package us.ihmc.euclid.geometry;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Represents an infinitely long 3D line defined by a 3D point and a 3D unitary vector.
 */
public class Line3D implements GeometryObject<Line3D>
{
   /** Coordinates of a point located on this line. */
   private final Point3D point = new Point3D();
   /** Normalized direction of this line. */
   private final Vector3D direction = new Vector3D();

   /**
    * Default constructor that initializes both {@link #point} and {@link #direction} to zero. This
    * point and vector have to be set to valid values to make this line usable.
    */
   public Line3D()
   {
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
    * Initializes this line to be passing through the given point, with the vector as the direction.
    * 
    * @param point point on this line. Not modified.
    * @param vector direction of this line. Not modified.
    */
   public Line3D(Point3DReadOnly point, Vector3DReadOnly vector)
   {
      set(point, vector);
   }

   /**
    * Initializes this line to be passing through the two given points.
    * 
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    */
   public Line3D(Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      set(firstPointOnLine, secondPointOnLine);
   }

   /**
    * Changes the point through which this line has to go.
    * 
    * @param point new point on this line. Not modified.
    */
   public void setPoint(Point3DReadOnly point)
   {
      this.point.set(point);
   }

   /**
    * Changes the direction of this line by setting to the normalized value of the given vector.
    * 
    * @param direction new direction of this line. Not modified.
    */
   public void setDirection(Vector3DReadOnly direction)
   {
      this.direction.set(direction);
      normalizeDirection();
   }

   /**
    * Redefines this line with a new point and a new direction vector.
    * 
    * @param point new point on this line. Not modified.
    * @param direction new direction of this line. Not modified.
    */
   public void set(Point3DReadOnly point, Vector3DReadOnly direction)
   {
      setPoint(point);
      setDirection(direction);
   }

   /**
    * Redefines this line such that it goes through the two given points.
    * 
    * @param firstPointOnLine first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    */
   public void set(Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      setPoint(firstPointOnLine);
      direction.sub(secondPointOnLine, firstPointOnLine);
      direction.normalize();
   }

   /**
    * Sets this line to be the same as the given line.
    * 
    * @param other the other line to copy. Not modified.
    */
   @Override
   public void set(Line3D other)
   {
      point.set(other.point);
      direction.set(other.direction);
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
    * Computes the minimum distance the given 3D point and this line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code direction.length() < Epsilons.ONE_TRILLIONTH}, this method returns the distance
    * between {@code point} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point 3D point to compute the distance from the line. Not modified.
    * @return the minimum distance between the 3D point and this 3D line.
    */
   public double distance(Point3DReadOnly point)
   {
      return EuclidGeometryTools.distanceFromPoint3DToLine3D(point, this.point, this.direction);
   }

   /**
    * This methods computes the minimum distance between this line and {@code otherLine}.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    * 
    * @param otherLine the other line to compute the distance from. Not modified.
    * @return the minimum distance between the two lines.
    */
   public double distance(Line3D otherLine)
   {
      return EuclidGeometryTools.distanceBetweenTwoLine3Ds(point, direction, otherLine.point, otherLine.direction);
   }

   /**
    * This methods computes two points P &in; this line and Q &in; {@code otherLine} such that the
    * distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    * 
    * @param otherLine the second line. Not modified.
    * @param closestPointOnThisLineToPack the 3D coordinates of the point P are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @param closestPointOnOtherLineToPack the 3D coordinates of the point Q are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    */
   public double closestPointsWith(Line3D otherLine, Point3DBasics closestPointOnThisLineToPack, Point3DBasics closestPointOnOtherLineToPack)
   {
      return EuclidGeometryTools.closestPoint3DsBetweenTwoLine3Ds(point, direction, otherLine.point, otherLine.direction, closestPointOnThisLineToPack,
                                                                  closestPointOnOtherLineToPack);
   }

   /**
    * Gets the reference to the point through which this line is going.
    * 
    * @return the reference to the point..
    */
   public Point3D getPoint()
   {
      return point;
   }

   /**
    * Gets the reference to the direction of this line.
    * 
    * @return the reference to the direction.
    */
   public Vector3D getDirection()
   {
      return direction;
   }

   /**
    * Gets the point defining this line by storing its coordinates in the given argument
    * {@code pointToPack}.
    * 
    * @param pointToPack point in which the coordinates of this line's point are stored. Modified.
    */
   public void getPoint(Point3DBasics pointToPack)
   {
      pointToPack.set(point);
   }

   /**
    * Gets the direction defining this line by storing its components in the given argument
    * {@code directionToPack}.
    * 
    * @param directionToPack vector in which the components of this line's direction are stored.
    *           Modified.
    */
   public void getDirection(Vector3DBasics directionToPack)
   {
      directionToPack.set(direction);
   }

   /**
    * Normalize the vector of this line.
    * <p>
    * Edge case:
    * <ul>
    * <li>if {@code direction.lengthSquared() < } {@link EuclidGeometryTools#ONE_TRILLIONTH}, the
    * direction is set to {@link Double#NaN}.
    * </ul>
    * </p>
    */
   private void normalizeDirection()
   {
      if (direction.containsNaN())
         return;

      double lengthSquared = direction.lengthSquared();

      if (lengthSquared < EuclidGeometryTools.ONE_TRILLIONTH)
         direction.setToNaN();
      else
         direction.scale(1.0 / EuclidCoreTools.fastSquareRoot(lengthSquared));
   }

   /**
    * Transforms this line using the given homogeneous transformation matrix.
    * 
    * @param transform the transform to apply on this line's point and vector. Not modified.
    */
   @Override
   public void applyTransform(Transform transform)
   {
      point.applyTransform(transform);
      direction.applyTransform(transform);
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
    */
   @Override
   public boolean epsilonEquals(Line3D other, double epsilon)
   {
      if (!point.epsilonEquals(other.point, epsilon))
         return false;
      if (!direction.epsilonEquals(other.direction, epsilon))
         return false;

      return true;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(Line3D)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
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

   /**
    * Tests on a per component basis, if this line 3D is exactly equal to {@code other}.
    *
    * @param other the other line 3D to compare against this. Not modified.
    * @return {@code true} if the two lines are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   public boolean equals(Line3D other)
   {
      if (other == null)
         return false;
      else
         return point.equals(other.point) && direction.equals(other.direction);
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
      return "Line 3D: point = " + point + ", direction = " + direction;
   }
}
