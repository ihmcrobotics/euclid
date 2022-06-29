package us.ihmc.euclid.geometry;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Line3DBasics;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Represents an infinitely long 3D line defined by a 3D point and a 3D unitary vector.
 */
public class Line3D implements Line3DBasics
{
   /** Coordinates of a point located on this line. */
   private final Point3D point = new Point3D();
   /** Normalized direction of this line. */
   private final UnitVector3D direction = new UnitVector3D(Axis3D.X);

   /**
    * Default constructor that initializes its {@code point} to zero and {@code direction} to
    * {@link Axis3D#X}.
    */
   public Line3D()
   {
   }

   /**
    * Initializes this line to be passing through the given point, with the vector as the direction.
    *
    * @param pointOnLineX   the x-coordinate of a point on this line.
    * @param pointOnLineY   the y-coordinate of a point on this line.
    * @param pointOnLineZ   the z-coordinate of a point on this line.
    * @param lineDirectionX the x-component of the direction of this line.
    * @param lineDirectionY the y-component of the direction of this line.
    * @param lineDirectionZ the z-component of the direction of this line.
    */
   public Line3D(double pointOnLineX, double pointOnLineY, double pointOnLineZ, double lineDirectionX, double lineDirectionY, double lineDirectionZ)
   {
      set(pointOnLineX, pointOnLineY, pointOnLineZ, lineDirectionX, lineDirectionY, lineDirectionZ);
   }

   /**
    * Creates a new line 3D and initializes it to {@code other}.
    *
    * @param line2DReadOnly the other line used to initialize this line. Not modified.
    */
   public Line3D(Line2DReadOnly line2DReadOnly)
   {
      set(line2DReadOnly);
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
    * Creates a new line and initializes it to go through the endpoints of the given line segment.
    *
    * @param lineSegment2DReadOnly the line segment used to initialize this line. Not modified.
    */
   public Line3D(LineSegment2DReadOnly lineSegment2DReadOnly)
   {
      set(lineSegment2DReadOnly);
   }

   /**
    * Creates a new line and initializes it to go through the endpoints of the given line segment.
    *
    * @param lineSegment3DReadOnly the line segment used to initialize this line. Not modified.
    */
   public Line3D(LineSegment3DReadOnly lineSegment3DReadOnly)
   {
      set(lineSegment3DReadOnly);
   }

   /**
    * Initializes this line to be passing through the two given points.
    *
    * @param firstPointOnLine  first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    * @throws RuntimeException if the two given points are exactly equal.
    */
   public Line3D(Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      set(firstPointOnLine, secondPointOnLine);
   }

   /**
    * Initializes this line to be passing through the given point, with the vector as the direction.
    *
    * @param pointOnLine   point on this line. Not modified.
    * @param lineDirection direction of this line. Not modified.
    */
   public Line3D(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      set(pointOnLine, lineDirection);
   }

   /** {@inheritDoc} */
   @Override
   public UnitVector3DBasics getDirection()
   {
      return direction;
   }

   /** {@inheritDoc} */
   @Override
   public Point3DBasics getPoint()
   {
      return point;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(Line3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Line3DReadOnly)
         return equals((Line3DReadOnly) object);
      else
         return false;
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
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this line 3D.
    *
    * @return the hash code value for this line 3D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(point, direction);
   }
}
