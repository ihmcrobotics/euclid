package us.ihmc.euclid.geometry;

import us.ihmc.euclid.geometry.interfaces.LineSegment2DBasics;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

/**
 * Represents a finite-length 2D line segment defined by its two 2D endpoints.
 */
public class LineSegment2D implements LineSegment2DBasics, GeometryObject<LineSegment2D>
{
   /** The first endpoint defining this line segment. */
   private final Point2D firstEndpoint = new Point2D();
   /** The second endpoint defining this line segment. */
   private final Point2D secondEndpoint = new Point2D();

   /**
    * Default constructor that initializes both endpoints of this line segment to zero.
    */
   public LineSegment2D()
   {
   }

   /**
    * Initializes this line segment to have the given endpoints.
    *
    * @param firstEndpointX x-coordinate of the first endpoint of this line segment.
    * @param firstEndpointY y-coordinate of the first endpoint of this line segment.
    * @param secondEndpointX x-coordinate of the second endpoint of this line segment.
    * @param secondEndpointY y-coordinate of the second endpoint of this line segment.
    */
   public LineSegment2D(double firstEndpointX, double firstEndpointY, double secondEndpointX, double secondEndpointY)
   {
      set(firstEndpointX, firstEndpointY, secondEndpointX, secondEndpointY);
   }

   /**
    * Creates a new line segment 2D and initializes it to {@code other}.
    *
    * @param other the other line segment used to initialize this line segment. Not modified.
    */
   public LineSegment2D(LineSegment2D other)
   {
      set(other);
   }

   /**
    * Creates a new line segment 2D and initializes it to {@code other}.
    *
    * @param other the other line segment used to initialize this line segment. Not modified.
    */
   public LineSegment2D(LineSegment2DReadOnly other)
   {
      set(other);
   }

   /**
    * Initializes this line segment to have the given endpoints.
    *
    * @param firstEndpoint the first endpoint of this line segment. Not modified.
    * @param secondEndpoint the second endpoint of this line segment. Not modified.
    */
   public LineSegment2D(Point2DReadOnly firstEndpoint, Point2DReadOnly secondEndpoint)
   {
      set(firstEndpoint, secondEndpoint);
   }

   /**
    * Sets this line segment to be same as the given line segment.
    *
    * @param other the other line segment to copy. Not modified.
    */
   @Override
   public void set(LineSegment2D other)
   {
      LineSegment2DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public Point2DBasics getFirstEndpoint()
   {
      return firstEndpoint;
   }

   /** {@inheritDoc} */
   @Override
   public Point2DBasics getSecondEndpoint()
   {
      return secondEndpoint;
   }

   /**
    * Tests on a per-component basis on both endpoints if this line segment is equal to {@code other}
    * with the tolerance {@code epsilon}.
    *
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two line segments are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(LineSegment2D other, double epsilon)
   {
      return LineSegment2DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two line segments are geometrically
    * similar.
    * <p>
    * The comparison is based on comparing the line segments' endpoints. Two line segments are
    * considered geometrically equal even if they are defined with opposite direction.
    * </p>
    *
    * @param other the line segment to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two line segments represent the same geometry, {@code false}
    *         otherwise.
    */
   @Override
   public boolean geometricallyEquals(LineSegment2D other, double epsilon)
   {
      return LineSegment2DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(LineSegment2DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((LineSegment2DReadOnly) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Provides a {@code String} representation of this line segment 2D as follows:<br>
    * Line segment 2D: 1st endpoint = (x, y), 2nd endpoint = (x, y)
    *
    * @return the {@code String} representing this line segment 2D.
    */
   @Override
   public String toString()
   {
      return EuclidGeometryIOTools.getLineSegment2DString(this);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this line segment
    * 2D.
    *
    * @return the hash code value for this line segment 2D.
    */
   @Override
   public int hashCode()
   {
      long bits = 31L * firstEndpoint.hashCode() + secondEndpoint.hashCode();
      return (int) (bits ^ bits >> 32);
   }
}
