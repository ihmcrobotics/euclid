package us.ihmc.euclid.geometry;

import us.ihmc.euclid.Axis2D;
import us.ihmc.euclid.geometry.interfaces.Line2DBasics;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.UnitVector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.UnitVector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

/**
 * Represents an infinitely-long 2D line defined by a 2D point and a 2D unit-vector.
 */
public class Line2D implements Line2DBasics
{
   /** Coordinates of a point located on this line. */
   private final Point2D point = new Point2D();
   /** Normalized direction of this line. */
   private final UnitVector2D direction = new UnitVector2D(Axis2D.X);

   /**
    * Default constructor that initializes its {@code point} to zero and {@code direction} to
    * {@link Axis2D#X}.
    */
   public Line2D()
   {
   }

   /**
    * Initializes this line to be passing through the given point, with the vector as the direction.
    *
    * @param pointOnLineX   the x-coordinate of a point on this line.
    * @param pointOnLineY   the y-coordinate of a point on this line.
    * @param lineDirectionX the x-component of the direction of this line.
    * @param lineDirectionY the y-component of the direction of this line.
    */
   public Line2D(double pointOnLineX, double pointOnLineY, double lineDirectionX, double lineDirectionY)
   {
      set(pointOnLineX, pointOnLineY, lineDirectionX, lineDirectionY);
   }

   /**
    * Creates a new line 2D and initializes it to {@code other}.
    *
    * @param other the other line used to initialize this line. Not modified.
    */
   public Line2D(Line2DReadOnly other)
   {
      set(other);
   }

   /**
    * Creates a new line and initializes it to go through the endpoints of the given line segment.
    *
    * @param lineSegment2DReadOnly the line segment used to initialize this line. Not modified.
    */
   public Line2D(LineSegment2DReadOnly lineSegment2DReadOnly)
   {
      set(lineSegment2DReadOnly);
   }

   /**
    * Initializes this line to be passing through the two given points.
    *
    * @param firstPointOnLine  first point on this line. Not modified.
    * @param secondPointOnLine second point on this line. Not modified.
    */
   public Line2D(Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      set(firstPointOnLine, secondPointOnLine);
   }

   /**
    * Initializes this line to be passing through the given point, with the vector as the direction.
    *
    * @param pointOnLine   point on this line. Not modified.
    * @param lineDirection direction of this line. Not modified.
    */
   public Line2D(Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      set(pointOnLine, lineDirection);
   }

   /**
    * Gets the read-only reference to the direction of this line.
    *
    * @return the reference to the direction.
    */
   @Override
   public UnitVector2DBasics getDirection()
   {
      return direction;
   }

   /**
    * Gets the read-only reference to the point through which this line is going.
    *
    * @return the reference to the point.
    */
   @Override
   public Point2DBasics getPoint()
   {
      return point;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(Line2DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Line2DReadOnly)
         return equals((Line2DReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this line 2D as follows:<br>
    * Line 3D: point = (x, y), direction = (x, y)
    *
    * @return the {@code String} representing this line 2D.
    */
   @Override
   public String toString()
   {
      return EuclidGeometryIOTools.getLine2DString(this);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this line 2D.
    *
    * @return the hash code value for this line 2D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(point, direction);
   }
}
