package us.ihmc.euclid.geometry;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DBasics;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

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
    * Initializes this line segment to have the given endpoints.
    *
    * @param endpoints a two-element array containing in order the first and second endpoints for
    *           this line segment. Not modified.
    * @throws IllegalArgumentException if the given array has a length different than 2.
    */
   public LineSegment2D(Point2DReadOnly[] endpoints)
   {
      set(endpoints);
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
      return getFirstEndpoint().containsNaN() || getSecondEndpoint().containsNaN();
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
    * Sets both endpoints of this line segment to zero.
    */
   @Override
   public void setToZero()
   {
      firstEndpoint.setToZero();
      secondEndpoint.setToZero();
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

   /**
    * Transforms this line segment using the given homogeneous transformation matrix.
    *
    * @param transform the transform to apply on the endpoints of this line segment. Not modified.
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a
    *            transformation in the XY-plane.
    */
   @Override
   public void applyTransform(Transform transform)
   {
      firstEndpoint.applyTransform(transform);
      secondEndpoint.applyTransform(transform);
   }

   /**
    * Transforms this line segment using the inverse of the given homogeneous transformation matrix.
    *
    * @param transform the transform to apply on the endpoints of this line segment. Not modified.
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a
    *            transformation in the XY-plane.
    */
   @Override
   public void applyInverseTransform(Transform transform)
   {
      firstEndpoint.applyInverseTransform(transform);
      secondEndpoint.applyInverseTransform(transform);
   }

   /**
    * Transforms this line segment using the given homogeneous transformation matrix and project the
    * result onto the XY-plane.
    *
    * @param transform the transform to apply on this line segment's endpoints. Not modified.
    */
   public void applyTransformAndProjectToXYPlane(Transform transform)
   {
      firstEndpoint.applyTransform(transform, false);
      secondEndpoint.applyTransform(transform, false);
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
   public boolean epsilonEquals(LineSegment2D other, double epsilon)
   {
      return firstEndpoint.epsilonEquals(other.firstEndpoint, epsilon) && secondEndpoint.epsilonEquals(other.secondEndpoint, epsilon);
   }

   /**
    * Tests on a per component basis, if this line segment 2D is exactly equal to {@code other}.
    *
    * @param other the other line segment 2D to compare against this. Not modified.
    * @return {@code true} if the two line segments are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   public boolean equals(LineSegment2D other)
   {
      if (other == null)
         return false;
      else
         return firstEndpoint.equals(other.firstEndpoint) && secondEndpoint.equals(other.secondEndpoint);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(LineSegment2D)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object obj)
   {
      try
      {
         return equals((LineSegment2D) obj);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Redefines this line segment with a new first endpoint and a vector going from the first to the
    * second endpoint.
    *
    * @param firstEndpoint new first endpoint. Not modified.
    * @param fromFirstToSecondEndpoint vector going from the first to the second endpoint. Not
    *           modified.
    */
   public void set(Point2DReadOnly firstEndpoint, Vector2DReadOnly fromFirstToSecondEndpoint)
   {
      this.firstEndpoint.set(firstEndpoint);
      secondEndpoint.add(firstEndpoint, fromFirstToSecondEndpoint);
   }

   /** {@inheritDoc} */
   @Override
   public void shift(boolean shiftToLeft, double distanceToShift)
   {
      double vectorX = secondEndpoint.getX() - firstEndpoint.getX();
      double vectorY = secondEndpoint.getY() - firstEndpoint.getY();

      double length = length();
      double orthogonalVectorX = -vectorY / length;
      double orthogonalVectorY = vectorX / length;

      if (!shiftToLeft)
      {
         orthogonalVectorX = -orthogonalVectorX;
         orthogonalVectorY = -orthogonalVectorY;
      }

      orthogonalVectorX = distanceToShift * orthogonalVectorX;
      orthogonalVectorY = distanceToShift * orthogonalVectorY;

      translate(orthogonalVectorX, orthogonalVectorY);
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
      if (firstEndpoint.geometricallyEquals(other.firstEndpoint, epsilon) && secondEndpoint.geometricallyEquals(other.secondEndpoint, epsilon))
         return true;
      if (firstEndpoint.geometricallyEquals(other.secondEndpoint, epsilon) && secondEndpoint.geometricallyEquals(other.firstEndpoint, epsilon))
         return true;
      return false;
   }

   /** {@inheritDoc} */
   @Override
   public Point2DReadOnly getFirstEndpoint()
   {
      return firstEndpoint;
   }

   /** {@inheritDoc} */
   @Override
   public Point2DReadOnly getSecondEndpoint()
   {
      return secondEndpoint;
   }

   @Override
   public void setFirstEndpoint(double firstEndpointX, double firstEndpointY)
   {
      firstEndpoint.set(firstEndpointX, firstEndpointY);
   }

   @Override
   public void setSecondEndpoint(double secondEndpointX, double secondEndpointY)
   {
      secondEndpoint.set(secondEndpointX, secondEndpointY);
   }
}
