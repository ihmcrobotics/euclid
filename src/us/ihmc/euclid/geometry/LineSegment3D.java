package us.ihmc.euclid.geometry;

import us.ihmc.euclid.geometry.interfaces.LineSegment3DBasics;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Represents a finite-length 3D line segment defined by its two 3D endpoints.
 */
public class LineSegment3D implements LineSegment3DBasics, GeometryObject<LineSegment3D>
{
   /** The first endpoint defining this line segment. */
   private final Point3D firstEndpoint = new Point3D();
   /** The second endpoint defining this line segment. */
   private final Point3D secondEndpoint = new Point3D();

   /**
    * Default constructor that initializes both endpoints of this line segment to zero.
    */
   public LineSegment3D()
   {
   }

   /**
    * Creates a new line segment 3D and initializes it to {@code other}.
    * 
    * @param other the other line segment used to initialize this line segment. Not modified.
    */
   public LineSegment3D(LineSegment3D other)
   {
      set(other);
   }

   /**
    * Creates a new line segment 3D and initializes it to {@code other}.
    * 
    * @param other the other line segment used to initialize this line segment. Not modified.
    */
   public LineSegment3D(LineSegment3DReadOnly other)
   {
      set(other);
   }

   /**
    * Initializes this line segment to have the given endpoints.
    * 
    * @param firstEndpoint the first endpoint of this line segment. Not modified.
    * @param secondEndpoint the second endpoint of this line segment. Not modified.
    */
   public LineSegment3D(Point3DReadOnly firstEndpoint, Point3DReadOnly secondEndpoint)
   {
      set(firstEndpoint, secondEndpoint);
   }

   /**
    * Initializes this line segment to have the given endpoints.
    * 
    * @param firstEndpointX x-coordinate of the first endpoint of this line segment.
    * @param firstEndpointY y-coordinate of the first endpoint of this line segment.
    * @param firstEndpointZ z-coordinate of the first endpoint of this line segment.
    * @param secondEndpointX x-coordinate of the second endpoint of this line segment.
    * @param secondEndpointY y-coordinate of the second endpoint of this line segment.
    * @param secondEndpointZ z-coordinate of the second endpoint of this line segment.
    */
   public LineSegment3D(double firstEndpointX, double firstEndpointY, double firstEndpointZ, double secondEndpointX, double secondEndpointY,
                        double secondEndpointZ)
   {
      set(firstEndpointX, firstEndpointY, firstEndpointZ, secondEndpointX, secondEndpointY, secondEndpointZ);
   }

   /**
    * Sets this line segment to be same as the given line segment.
    * 
    * @param other the other line segment to copy. Not modified.
    */
   @Override
   public void set(LineSegment3D other)
   {
      LineSegment3DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public Point3DBasics getFirstEndpoint()
   {
      return firstEndpoint;
   }

   /** {@inheritDoc} */
   @Override
   public Point3DBasics getSecondEndpoint()
   {
      return secondEndpoint;
   }

   /**
    * Tests on a per-component basis on both endpoints if this line segment is equal to
    * {@code other} with the tolerance {@code epsilon}.
    * 
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two line segments are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(LineSegment3D other, double epsilon)
   {
      return LineSegment3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(LineSegment3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object obj)
   {
      try
      {
         return equals((LineSegment3DReadOnly) obj);
      }
      catch (ClassCastException e)
      {
         return false;
      }
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
   public boolean geometricallyEquals(LineSegment3D other, double epsilon)
   {
      return LineSegment3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Provides a {@code String} representation of this line segment 3D as follows:<br>
    * Line segment 3D: 1st endpoint = (x, y, z), 2nd endpoint = (x, y, z)
    *
    * @return the {@code String} representing this line segment 3D.
    */
   @Override
   public String toString()
   {
      return EuclidGeometryIOTools.getLineSegment3DString(this);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this line segment
    * 3D.
    *
    * @return the hash code value for this line segment 3D.
    */
   @Override
   public int hashCode()
   {
      long bits = 31L * firstEndpoint.hashCode() + secondEndpoint.hashCode();
      return (int) (bits ^ bits >> 32);
   }
}
