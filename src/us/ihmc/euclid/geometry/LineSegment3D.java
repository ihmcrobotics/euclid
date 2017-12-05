package us.ihmc.euclid.geometry;

import us.ihmc.euclid.geometry.interfaces.LineSegment3DBasics;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.*;

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
    * Initializes this line segment to have the given endpoints.
    * 
    * @param endpoints a two-element array containing in order the first and second endpoints for
    *           this line segment. Not modified.
    * @throws IllegalArgumentException if the given array has a length different than 2.
    */
   public LineSegment3D(Point3DReadOnly[] endpoints)
   {
      set(endpoints);
   }

   /** {@inheritDoc} */
   @Override
   public void setFirstEndpoint(double firstEndpointX, double firstEndpointY, double firstEndpointZ)
   {
      firstEndpoint.set(firstEndpointX, firstEndpointY, firstEndpointZ);
   }

   /** {@inheritDoc} */
   @Override
   public void setSecondEndpoint(double secondEndpointX, double secondEndpointY, double secondEndpointZ)
   {
      secondEndpoint.set(secondEndpointX, secondEndpointY, secondEndpointZ);
   }

   /**
    * Redefines this line segment with a new first endpoint and a vector going from the first to the
    * second endpoint.
    *
    * @param firstEndpoint new first endpoint. Not modified.
    * @param fromFirstToSecondEndpoint vector going from the first to the second endpoint. Not
    *           modified.
    */
   public void set(Point3DReadOnly firstEndpoint, Vector3DReadOnly fromFirstToSecondEndpoint)
   {
      this.firstEndpoint.set(firstEndpoint);
      this.secondEndpoint.add(firstEndpoint, fromFirstToSecondEndpoint);
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

   /**
    * Sets both endpoints of this line segment to zero.
    */
   @Override
   public void setToZero()
   {
      firstEndpoint.setToZero();
      secondEndpoint.setToZero();
   }

   /**
    * Sets both endpoints of this line segment to {@link Double#NaN}. After calling this method,
    * this line segment becomes invalid. A new pair of valid endpoints will have to be set so this
    * line segment is again usable.
    */
   @Override
   public void setToNaN()
   {
      firstEndpoint.setToNaN();
      secondEndpoint.setToNaN();
   }

   /**
    * Tests if this line segment contains {@link Double#NaN}.
    * 
    * @return {@code true} if {@link #firstEndpoint} and/or {@link #secondEndpoint} contains
    *         {@link Double#NaN}, {@code false} otherwise.
    */
   @Override
   public boolean containsNaN()
   {
      return firstEndpoint.containsNaN() || secondEndpoint.containsNaN();
   }

   /**
    * Transforms this line segment using the given homogeneous transformation matrix.
    * 
    * @param transform the transform to apply on the endpoints of this line segment. Not modified.
    */
   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(firstEndpoint);
      transform.transform(secondEndpoint);
   }

   /**
    * Transforms this line segment using the inverse of the given homogeneous transformation matrix.
    * 
    * @param transform the transform to apply on the endpoints of this line segment. Not modified.
    */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(firstEndpoint);
      transform.inverseTransform(secondEndpoint);
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
      return firstEndpoint.epsilonEquals(other.firstEndpoint, epsilon) && secondEndpoint.epsilonEquals(other.secondEndpoint, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(LineSegment3D)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object obj)
   {
      try
      {
         return equals((LineSegment3D) obj);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Tests on a per component basis, if this line segment 3D is exactly equal to {@code other}.
    *
    * @param other the other line segment 3D to compare against this. Not modified.
    * @return {@code true} if the two line segments are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   public boolean equals(LineSegment3D other)
   {
      if (other == null)
         return false;
      else
         return firstEndpoint.equals(other.firstEndpoint) && secondEndpoint.equals(other.secondEndpoint);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two line segments are
    * geometrically similar.
    * <p>
    * The comparison is based on comparing the line segments' endpoints. Two line segments are
    * considered geometrically equal even if they are defined with opposite direction.
    * </p>
    *
    * @param other the line segment to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two line segments represent the same geometry, {@code false}
    *            otherwise.
    */
   @Override
   public boolean geometricallyEquals(LineSegment3D other, double epsilon)
   {
      if (firstEndpoint.geometricallyEquals(other.firstEndpoint, epsilon) && secondEndpoint.geometricallyEquals(other.secondEndpoint, epsilon))
         return true;
      if (firstEndpoint.geometricallyEquals(other.secondEndpoint, epsilon) && secondEndpoint.geometricallyEquals(other.firstEndpoint, epsilon))
         return true;
      return false;
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

   @Override
   public Point3DReadOnly getFirstEndpoint()
   {
      return firstEndpoint;
   }

   @Override
   public Point3DReadOnly getSecondEndpoint()
   {
      return secondEndpoint;
   }
}
