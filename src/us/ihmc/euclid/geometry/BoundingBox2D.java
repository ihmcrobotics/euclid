package us.ihmc.euclid.geometry;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.geometry.exceptions.BoundingBoxException;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

/**
 * A {@link BoundingBox2D} can be used to defines from its minimum and maximum coordinates an
 * axis-aligned bounding box in the XY-plane.
 */
public class BoundingBox2D implements GeometryObject<BoundingBox2D>
{
   /** The minimum coordinates of this bounding box. */
   private final Point2D minPoint = new Point2D();
   /** The maximum coordinates of this bounding box. */
   private final Point2D maxPoint = new Point2D();

   /**
    * Creates a new bounding box 2D from its center coordinate {@code center} and a tuple 2D holding onto half its size {@code plusMinusTuple}.
    * <p>
    * The minimum and maximum coordinates of the resulting bounding box are calculated as follows:
    * <ul>
    * <li> {@code minPoint = center - plusMinusTuple}
    * <li> {@code maxPoint = center + plusMinusTuple}
    * </ul>
    * </p>
    * 
    * @param center the center coordinate of the new bounding box. Not modified.
    * @param plusMinusTuple tuple representing half of the size of the new bounding box. Not modified.
    * @return the new bounding box.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the
    *            maximum coordinate on the same axis.
    */
   public static BoundingBox2D createUsingCenterAndPlusMinusVector(Point2DReadOnly center, Tuple2DReadOnly plusMinusTuple)
   {
      Point2D minPoint = new Point2D();
      Point2D maxPoint = new Point2D();

      minPoint.sub(center, plusMinusTuple);
      maxPoint.add(center, plusMinusTuple);

      return new BoundingBox2D(minPoint, maxPoint);
   }

   /**
    * Creates a new bounding box such that it is the smallest bounding box containing the two given bounding boxes {@code boundingBoxOne} and {@code boundingBoxTwo}.
    * 
    * @param boundingBoxOne the first bounding box. Not modified.
    * @param boundingBoxTwo the second bounding box. Not modified.
    * @return the new bounding box.
    */
   public static BoundingBox2D union(BoundingBox2D boundingBoxOne, BoundingBox2D boundingBoxTwo)
   {
      double minX = Math.min(boundingBoxOne.minPoint.getX(), boundingBoxTwo.minPoint.getX());
      double minY = Math.min(boundingBoxOne.minPoint.getY(), boundingBoxTwo.minPoint.getY());

      double maxX = Math.max(boundingBoxOne.maxPoint.getX(), boundingBoxTwo.maxPoint.getX());
      double maxY = Math.max(boundingBoxOne.maxPoint.getY(), boundingBoxTwo.maxPoint.getY());

      Point2D unionMin = new Point2D(minX, minY);
      Point2D unionMax = new Point2D(maxX, maxY);

      return new BoundingBox2D(unionMin, unionMax);
   }

   /**
    * Creates a new bounding box initialized with both its minimum and maximum coordinates to (0,
    * 0).
    */
   public BoundingBox2D()
   {
   }

   /**
    * Creates a new bounding box and initializes it to the given minimum and maximum coordinates.
    *
    * @param min the minimum coordinates for this. Not modified.
    * @param max the maximum coordinates for this. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the
    *            maximum coordinate on the same axis.
    */
   public BoundingBox2D(Point2DReadOnly min, Point2DReadOnly max)
   {
      set(min, max);
   }

   /**
    * Creates a new bounding box and initializes it to {@code other}.
    *
    * @param other the other bounding box used to initialize this. Not modified.
    */
   public BoundingBox2D(BoundingBox2D other)
   {
      set(other);
   }

   /**
    * Creates a new bounding box and initializes it to the given minimum and maximum coordinates.
    *
    * @param min the minimum coordinates for this. Not modified.
    * @param max the maximum coordinates for this. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the
    *            maximum coordinate on the same axis.
    */
   public BoundingBox2D(double[] min, double[] max)
   {
      set(min, max);
   }

   /**
    * Creates a new bounding box and initializes it to the given minimum and maximum coordinates.
    *
    * @param minX the minimum x-coordinate for this.
    * @param minY the minimum y-coordinate for this.
    * @param maxX the maximum x-coordinates for this.
    * @param maxY the maximum y-coordinates for this.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the
    *            maximum coordinate on the same axis.
    */
   public BoundingBox2D(double minX, double minY, double maxX, double maxY)
   {
      set(minX, minY, maxX, maxX);
   }

   /**
    * Asserts that the minimum coordinates are less or equal to the maximum coordinates.
    *
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the
    *            maximum coordinate on the same axis.
    */
   public void checkBounds()
   {
      if (minPoint.getX() > maxPoint.getX())
         throw new BoundingBoxException("minPoint.getX() > maxPoint.getX(): " + minPoint.getX() + ">" + maxPoint.getX());
      if (minPoint.getY() > maxPoint.getY())
         throw new BoundingBoxException("minPoint.getY() > maxPoint.getY(): " + minPoint.getY() + ">" + maxPoint.getY());
   }

   /**
    * Sets the minimum coordinate of this bounding box.
    * 
    * @param min the minimum coordinate for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the
    *            maximum coordinate on the same axis.
    */
   public void setMin(Point2DReadOnly min)
   {
      minPoint.set(min);
      checkBounds();
   }

   /**
    * Sets the maximum coordinate of this bounding box.
    * 
    * @param max the maximum coordinate for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the
    *            maximum coordinate on the same axis.
    */
   public void setMax(Point2DReadOnly max)
   {
      maxPoint.set(max);
      checkBounds();
   }

   /**
    * Redefines this bounding box with new minimum and maximum coordinates.
    *
    * @param minX the new minimum x-coordinate for this bounding box.
    * @param minY the new minimum y-coordinate for this bounding box.
    * @param maxX the new maximum x-coordinates for this bounding box.
    * @param maxY the new maximum y-coordinates for this bounding box.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the
    *            maximum coordinate on the same axis.
    */
   public void set(double minX, double minY, double maxX, double maxY)
   {
      minPoint.set(minX, minY);
      maxPoint.set(maxX, maxY);
      checkBounds();
   }

   /**
    * Redefines this bounding box with new minimum and maximum coordinates.
    *
    * @param min the new minimum coordinates for this bounding box. Not modified.
    * @param max the new maximum coordinates for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the
    *            maximum coordinate on the same axis.
    */
   public void set(double[] min, double[] max)
   {
      minPoint.set(min);
      maxPoint.set(max);
      checkBounds();
   }

   /**
    * Redefines this bounding box with new minimum and maximum coordinates.
    *
    * @param min the new minimum coordinates for this bounding box. Not modified.
    * @param max the new maximum coordinates for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the
    *            maximum coordinate on the same axis.
    */
   public void set(Point2DReadOnly min, Point2DReadOnly max)
   {
      minPoint.set(min);
      maxPoint.set(max);
      checkBounds();
   }

   /**
    * Redefines this bounding box to be the same as the given {@code other}.
    * 
    * @param other the bounding box used to redefine this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the
    *            maximum coordinate on the same axis.
    */
   @Override
   public void set(BoundingBox2D other)
   {
      minPoint.set(other.minPoint);
      maxPoint.set(other.maxPoint);
      checkBounds();
   }

   /**
    * Invalidates this bounding box by setting all its coordinates to {@link Double#NaN}.
    */
   @Override
   public void setToNaN()
   {
      minPoint.setToNaN();
      maxPoint.setToNaN();
   }

   /**
    * Resets this bounding box by setting both its minimum and maximum coordinates to (0, 0).
    */
   @Override
   public void setToZero()
   {
      minPoint.setToZero();
      maxPoint.setToZero();
   }

   /** {@inheritDoc} */
   @Override
   public boolean containsNaN()
   {
      return minPoint.containsNaN() || maxPoint.containsNaN();
   }

   /**
    * Calculates the coordinate of the center of this bounding box and stores it in the given
    * {@code centerToPack}.
    * 
    * @param centerToPack point 2D in which the center of this bounding box is stored. Modified.
    */
   public void getCenterPoint(Point2DBasics centerToPack)
   {
      centerToPack.interpolate(minPoint, maxPoint, 0.5);
   }

   /**
    * Performs a linear interpolation between this bounding box minimum and maximum coordinates
    * using independent parameters {@code xParameter} and {@code yParameter} for the x-axis and
    * y-axis respectively. The result is stored in {@code pointToPack}.
    * <p>
    * <ul>
    * <li>{@code (xParameter == 0)} results in: {@code (pointToPack.getX() == this.getMinX())}.
    * <li>{@code (xParameter == 1)} results in: {@code (pointToPack.getX() == this.getMaxX())}.
    * <li>{@code (yParameter == 0)} results in: {@code (pointToPack.getY() == this.getMinY())}.
    * <li>{@code (yParameter == 1)} results in: {@code (pointToPack.getY() == this.getMaxY())}.
    * </ul>
    * </p>
    * 
    * @param xParameter the parameter to use for the interpolation along the x-axis.
    * @param yParameter the parameter to use for the interpolation along the y-axis.
    * @param pointToPack the point 2D in which the result is stored. Modified.
    */
   public void getPointGivenParameters(double xParameter, double yParameter, Point2DBasics pointToPack)
   {
      pointToPack.setX(minPoint.getX() + xParameter * (maxPoint.getX() - minPoint.getX()));
      pointToPack.setY(minPoint.getY() + yParameter * (maxPoint.getY() - minPoint.getY()));
   }

   /**
    * Calculates the squared value of the distance between the minimum and maximum coordinates of
    * this bounding box.
    * 
    * @return the squared value of this bounding box diagonal.
    */
   public double getDiagonalLengthSquared()
   {
      return minPoint.distanceSquared(maxPoint);
   }

   /**
    * Tests if the {@code query} is located inside this bounding box.
    * <p>
    * The query is considered to be outside if located exactly on an edge of this bounding box.
    * </p>
    * 
    * @param query the query to test if it is located inside this bounding box. Not modified.
    * @return {@code true} if the query is inside, {@code false} if outside or located on an edge of
    *         this bounding box.
    */
   public boolean isInsideExclusive(Point2DReadOnly query)
   {
      if (query.getY() < minPoint.getY() || query.getY() > maxPoint.getY())
         return false;

      if (query.getX() < minPoint.getX() || query.getX() > maxPoint.getX())
         return false;

      return true;
   }

   /**
    * Tests if the {@code query} is located inside this bounding box.
    * <p>
    * The query is considered to be inside if located exactly on an edge of this bounding box.
    * </p>
    * 
    * @param query the query to test if it is located inside this bounding box. Not modified.
    * @return {@code true} if the query is inside or located on an edge of this bounding box,
    *         {@code false} if outside.
    */
   public boolean isInsideInclusive(Point2DReadOnly query)
   {
      if (query.getY() <= minPoint.getY() || query.getY() >= maxPoint.getY())
         return false;

      if (query.getX() <= minPoint.getX() || query.getX() >= maxPoint.getX())
         return false;

      return true;
   }

   /**
    * Tests if the {@code query} is located inside this bounding box given the tolerance
    * {@code epsilon}.
    * <p>
    * <ul>
    * <li>if {@code epsilon == 0}, this method is equivalent to
    * {@link #isInsideExclusive(Point2DReadOnly)}.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled up by shifting the edges
    * of {@code epsilon} toward the outside.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled down by shifting the edges
    * of {@code epsilon} toward the inside.
    * </ul>
    * </p>
    * 
    * @param query the query to test if it is located inside this bounding box. Not modified.
    * @param epsilon the tolerance to use for this test.
    * @return {@code true} if the query is considered to be inside the bounding box, {@code false}
    *         otherwise.
    */
   public boolean isInsideEpsilon(Point2DReadOnly query, double epsilon)
   {
      if (query.getY() < (minPoint.getY() - epsilon) || query.getY() > (maxPoint.getY() + epsilon))
         return false;

      if (query.getX() < (minPoint.getX() - epsilon) || query.getX() > (maxPoint.getX() + epsilon))
         return false;

      return true;
   }

   /**
    * Tests if this bounding box and {@code other} intersects.
    * <p>
    * The two bounding boxes are considered to be intersecting if they share a corner or an edge.
    * </p>
    * 
    * @param other the other bounding box to test if it is intersecting with this bounding box. Not
    *           Modified.
    * @return {@code true} if the two bounding boxes intersect, {@code false} otherwise.
    */
   public boolean intersectsInclusive(BoundingBox2D other)
   {
      if (other.minPoint.getX() > maxPoint.getX())
         return false;

      if (other.maxPoint.getX() < minPoint.getX())
         return false;

      if (other.minPoint.getY() > maxPoint.getY())
         return false;

      if (other.maxPoint.getY() < minPoint.getY())
         return false;

      return true;
   }

   /**
    * Tests if this bounding box and {@code other} intersects.
    * <p>
    * The two bounding boxes are considered to not be intersecting if they share a corner or an
    * edge.
    * </p>
    * 
    * @param other the other bounding box to test if it is intersecting with this bounding box. Not
    *           Modified.
    * @return {@code true} if the two bounding boxes intersect, {@code false} otherwise.
    */
   public boolean intersectsExclusive(BoundingBox2D other)
   {
      if (other.minPoint.getX() >= maxPoint.getX())
         return false;

      if (other.maxPoint.getX() <= minPoint.getX())
         return false;

      if (other.minPoint.getY() >= maxPoint.getY())
         return false;

      if (other.maxPoint.getY() <= minPoint.getY())
         return false;

      return true;
   }

   /**
    * Tests if this bounding box and {@code other} intersects.
    * <p>
    * <ul>
    * <li>if {@code epsilon == 0}, this method is equivalent to
    * {@link #intersectsEpsilon(BoundingBox2D)}.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled up by shifting the edges
    * of {@code epsilon} toward the outside.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled down by shifting the edges
    * of {@code epsilon} toward the inside.
    * </ul>
    * </p>
    * 
    * @param other the other bounding box to test if it is intersecting with this bounding box. Not
    *           Modified.
    * @param epsilon the tolerance to use in this test.
    * @return {@code true} if the two bounding boxes intersect, {@code false} otherwise.
    */
   public boolean intersectsEpsilon(BoundingBox2D other, double epsilon)
   {
      if (other.minPoint.getX() > maxPoint.getX() + epsilon)
         return false;

      if (other.maxPoint.getX() < minPoint.getX() - epsilon)
         return false;

      if (other.minPoint.getY() > maxPoint.getY() + epsilon)
         return false;

      if (other.maxPoint.getY() < minPoint.getY() - epsilon)
         return false;

      return true;
   }

   /**
    * Gets the minimum coordinate of this bounding box and stores it in the given {@code minToPack}.
    * 
    * @param minToPack point 2D in which the minimum coordinate of this bounding box is stored.
    *           Modified.
    */
   public void getMinPoint(Point2DBasics minToPack)
   {
      minToPack.set(minPoint);
   }

   /**
    * Gets the maximum coordinate of this bounding box and stores it in the given {@code maxToPack}.
    * 
    * @param maxToPack point 2D in which the maximum coordinate of this bounding box is stored.
    *           Modified.
    */
   public void getMaxPoint(Point2DBasics maxToPack)
   {
      maxToPack.set(maxPoint);
   }

   /**
    * Gets the minimum coordinate of this bounding box and stores it in the given array {@code maxToPack}.
    * 
    * @param maxToPack array in which the minimum coordinate of this bounding box is stored.
    *           Modified.
    */
   public void getMinPoint(double[] minToPack)
   {
      minPoint.get(minToPack);
   }

   /**
    * Gets the maximum coordinate of this bounding box and stores it in the given array {@code maxToPack}.
    * 
    * @param maxToPack array in which the maximum coordinate of this bounding box is stored.
    *           Modified.
    */
   public void getMaxPoint(double[] maxToPack)
   {
      maxPoint.get(maxToPack);
   }


   /**
    * Gets the read-only reference to the minimum coordinate of this bounding box.
    * 
    * @return the read-only reference to the minimum coordinate.
    */
   public Point2DReadOnly getMinPoint()
   {
      return minPoint;
   }

   /**
    * Gets the read-only reference to the maximum coordinate of this bounding box.
    * 
    * @return the read-only reference to the maximum coordinate.
    */
   public Point2DReadOnly getMaxPoint()
   {
      return maxPoint;
   }

   /**
    * Gets the minimum x-coordinate of this bounding box.
    * 
    * @return the minimum x-coordinate.
    */
   public double getMinX()
   {
      return minPoint.getX();
   }

   /**
    * Gets the minimum y-coordinate of this bounding box.
    * 
    * @return the minimum y-coordinate.
    */
   public double getMinY()
   {
      return minPoint.getY();
   }

   /**
    * Gets the maximum x-coordinate of this bounding box.
    * 
    * @return the maximum x-coordinate.
    */
   public double getMaxX()
   {
      return maxPoint.getX();
   }

   /**
    * Gets the maximum y-coordinate of this bounding box.
    * 
    * @return the maximum y-coordinate.
    */
   public double getMaxY()
   {
      return maxPoint.getY();
   }

   /**
    * Transforms this bounding box 2D using the given homogeneous transformation matrix.
    * 
    * @param transform the transform to apply on the minimum and maximum coordinates of this
    *           bounding box. Not modified.
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a
    *            transformation in the XY plane.
    */
   @Override
   public void applyTransform(Transform transform)
   {
      transform.transform(minPoint);
      transform.transform(maxPoint);
   }

   /**
    * Tests on a per-component basis on the minimum and maximum coordinates if this bounding box is
    * equal to {@code other} with the tolerance {@code epsilon}.
    * 
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two bounding boxes are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(BoundingBox2D other, double epsilon)
   {
      return minPoint.epsilonEquals(other.minPoint, epsilon) && maxPoint.epsilonEquals(other.maxPoint, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(BoundingBox2D)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object obj)
   {
      try
      {
         return equals((BoundingBox2D) obj);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Tests on a per component basis, if this bounding box 2D is exactly equal to {@code other}.
    *
    * @param other the other bounding box 2D to compare against this. Not modified.
    * @return {@code true} if the two bounding boxes are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   public boolean equals(BoundingBox2D other)
   {
      if (other == null)
         return false;
      else
         return minPoint.equals(other.minPoint) && maxPoint.equals(other.maxPoint);
   }

   /**
    * Provides a {@code String} representation of this bounding box 2D as follows:<br>
    * Bounding Box 2D: min = (x, y), max = (x, y)
    *
    * @return the {@code String} representing this bounding box 2D.
    */
   @Override
   public String toString()
   {
      return "Bounding Box 2D: " + "min = " + minPoint + ", max = " + maxPoint;
   }
}
