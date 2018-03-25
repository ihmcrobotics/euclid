package us.ihmc.euclid.geometry;

import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.GeometricallyComparable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

/**
 * A {@link BoundingBox2D} can be used to define from a set of minimum and maximum coordinates an
 * axis-aligned bounding box in the XY-plane.
 */
public class BoundingBox2D
      implements BoundingBox2DReadOnly, EpsilonComparable<BoundingBox2D>, Settable<BoundingBox2D>, Clearable, GeometricallyComparable<BoundingBox2D>
{
   /** The minimum coordinates of this bounding box. */
   private final Point2D minPoint = new Point2D();
   /** The maximum coordinates of this bounding box. */
   private final Point2D maxPoint = new Point2D();

   /**
    * Creates a new bounding box 2D from its center coordinate {@code center} and a tuple 2D holding
    * onto half its size {@code plusMinusTuple}.
    * <p>
    * The minimum and maximum coordinates of the resulting bounding box are calculated as follows:
    * <ul>
    * <li>{@code minPoint = center - plusMinusTuple}
    * <li>{@code maxPoint = center + plusMinusTuple}
    * </ul>
    * </p>
    *
    * @param center the center coordinate of the new bounding box. Not modified.
    * @param plusMinusTuple tuple representing half of the size of the new bounding box. Not modified.
    * @return the new bounding box.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   public static BoundingBox2D createUsingCenterAndPlusMinusVector(Point2DReadOnly center, Tuple2DReadOnly plusMinusTuple)
   {
      BoundingBox2D boundingBox2D = new BoundingBox2D();
      boundingBox2D.set(center, new Vector2D(plusMinusTuple));
      return boundingBox2D;
   }

   /**
    * Creates a new bounding box such that it is the smallest bounding box containing the two given
    * bounding boxes {@code boundingBoxOne} and {@code boundingBoxTwo}.
    *
    * @param boundingBoxOne the first bounding box. Not modified.
    * @param boundingBoxTwo the second bounding box. Not modified.
    * @return the new bounding box.
    */
   public static BoundingBox2D union(BoundingBox2D boundingBoxOne, BoundingBox2D boundingBoxTwo)
   {
      BoundingBox2D union = new BoundingBox2D();
      union.combine(boundingBoxOne, boundingBoxTwo);
      return union;
   }

   /**
    * Creates a new bounding box initialized with both its minimum and maximum coordinates to (0, 0).
    */
   public BoundingBox2D()
   {
   }

   /**
    * Creates a new bounding box and initializes it to the given minimum and maximum coordinates.
    *
    * @param min the minimum coordinates for this. Not modified.
    * @param max the maximum coordinates for this. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
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
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
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
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   public BoundingBox2D(double minX, double minY, double maxX, double maxY)
   {
      set(minX, minY, maxX, maxY);
   }

   /**
    * Sets the minimum coordinate of this bounding box.
    *
    * @param min the minimum coordinate for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   public void setMin(Point2DReadOnly min)
   {
      minPoint.set(min);
      checkBounds();
   }

   /**
    * Sets the minimum coordinate of this bounding box.
    *
    * @param min the minimum coordinates for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   public void setMin(double[] min)
   {
      minPoint.set(min);
      checkBounds();
   }

   /**
    * Sets the minimum coordinate of this bounding box.
    *
    * @param minX the new minimum x-coordinate for this bounding box.
    * @param minY the new minimum y-coordinate for this bounding box.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   public void setMin(double minX, double minY)
   {
      minPoint.set(minX, minY);
      checkBounds();
   }

   /**
    * Sets the maximum coordinate of this bounding box.
    *
    * @param max the maximum coordinate for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   public void setMax(Point2DReadOnly max)
   {
      maxPoint.set(max);
      checkBounds();
   }

   /**
    * Sets the maximum coordinate of this bounding box.
    *
    * @param max the maximum coordinates for this bounding box. Not modified.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   public void setMax(double[] max)
   {
      maxPoint.set(max);
      checkBounds();
   }

   /**
    * Sets the maximum coordinate of this bounding box.
    *
    * @param maxX the new maximum x-coordinate for this bounding box.
    * @param maxY the new maximum y-coordinate for this bounding box.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   public void setMax(double maxX, double maxY)
   {
      maxPoint.set(maxX, maxY);
      checkBounds();
   }

   /**
    * Redefines this bounding box with new minimum and maximum coordinates.
    *
    * @param minX the new minimum x-coordinate for this bounding box.
    * @param minY the new minimum y-coordinate for this bounding box.
    * @param maxX the new maximum x-coordinates for this bounding box.
    * @param maxY the new maximum y-coordinates for this bounding box.
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
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
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
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
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *            coordinate on the same axis.
    */
   public void set(Point2DReadOnly min, Point2DReadOnly max)
   {
      minPoint.set(min);
      maxPoint.set(max);
      checkBounds();
   }

   /**
    * Redefines this bounding box given its {@code center} location and half its size along each axis
    * {@code halfSize}.
    *
    * @param center the new center location of this bounding box. Not modified.
    * @param halfSize half the size of this bounding box. Not modified.
    */
   public void set(Point2DReadOnly center, Vector2DReadOnly halfSize)
   {
      minPoint.sub(center, halfSize);
      maxPoint.add(center, halfSize);
      checkBounds();
   }

   /**
    * Redefines this bounding box to be the same as the given {@code other}.
    *
    * @param other the bounding box used to redefine this bounding box. Not modified.
    */
   @Override
   public void set(BoundingBox2D other)
   {
      minPoint.set(other.minPoint);
      maxPoint.set(other.maxPoint);
   }

   /**
    * Combines this bounding box with {@code other} such that it becomes the smallest bounding box
    * containing this and {@code other}.
    *
    * @param other the other bounding box to combine with this. Not modified.
    */
   public void combine(BoundingBox2D other)
   {
      combine(this, other);
   }

   /**
    * Sets this bounding box to the union of {@code boundingBoxOne} and {@code boundingBoxTwo}.
    * <p>
    * This bounding box is set such that it is the smallest bounding box containing the two given
    * bounding boxes.
    * </p>
    *
    * @param boundingBoxOne the first bounding box. Can be the same instance as this. Not modified.
    * @param boundingBoxTwo the second bounding box. Can be the same instance as this. Not modified.
    */
   public void combine(BoundingBox2D boundingBoxOne, BoundingBox2D boundingBoxTwo)
   {
      double minX = Math.min(boundingBoxOne.getMinX(), boundingBoxTwo.getMinX());
      double minY = Math.min(boundingBoxOne.getMinY(), boundingBoxTwo.getMinY());

      double maxX = Math.max(boundingBoxOne.getMaxX(), boundingBoxTwo.getMaxX());
      double maxY = Math.max(boundingBoxOne.getMaxY(), boundingBoxTwo.getMaxY());

      set(minX, minY, maxX, maxY);
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

   @Override
   public boolean containsNaN()
   {
      return BoundingBox2DReadOnly.super.containsNaN();
   }

   /**
    * Updates this bounding box to be the smallest bounding box that includes this and the given point.
    *
    * @param point the point to be included in this bounding box. Not modified.
    */
   public void updateToIncludePoint(Point2DReadOnly point)
   {
      this.updateToIncludePoint(point.getX(), point.getY());
   }

   /**
    * Updates this bounding box to be the smallest bounding box that includes this and the given point.
    *
    * @param x x-coordinate of the point to be included in this bounding box. Not modified.
    * @param y y-coordinate of the point to be included in this bounding box. Not modified.
    */
   public void updateToIncludePoint(double x, double y)
   {
      if (Double.isNaN(minPoint.getX()) || x < minPoint.getX())
      {
         minPoint.setX(x);
      }

      if (Double.isNaN(minPoint.getY()) || y < minPoint.getY())
      {
         minPoint.setY(y);
      }

      if (Double.isNaN(maxPoint.getX()) || x > maxPoint.getX())
      {
         maxPoint.setX(x);
      }

      if (Double.isNaN(maxPoint.getY()) || y > maxPoint.getY())
      {
         maxPoint.setY(y);
      }
   }

   /**
    * Gets the read-only reference to the minimum coordinate of this bounding box.
    *
    * @return the read-only reference to the minimum coordinate.
    */
   @Override
   public Point2DReadOnly getMinPoint()
   {
      return minPoint;
   }

   /**
    * Gets the read-only reference to the maximum coordinate of this bounding box.
    *
    * @return the read-only reference to the maximum coordinate.
    */
   @Override
   public Point2DReadOnly getMaxPoint()
   {
      return maxPoint;
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
      return BoundingBox2DReadOnly.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(BoundingBox2D)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((BoundingBox2D) object);
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
      return BoundingBox2DReadOnly.super.equals(other);
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
      return EuclidGeometryIOTools.getBoundingBox2DString(this);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two bounding boxes are geometrically
    * similar, i.e. the distance between their min and max points is less than or equal to
    * {@code epsilon}.
    *
    * @param other the bounding box to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two bounding boxes represent the same geometry, {@code false}
    *         otherwise.
    */
   @Override
   public boolean geometricallyEquals(BoundingBox2D other, double epsilon)
   {
      return BoundingBox2DReadOnly.super.geometricallyEquals(other, epsilon);
   }
}
