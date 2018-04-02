package us.ihmc.euclid.geometry;

import us.ihmc.euclid.geometry.interfaces.BoundingBox2DBasics;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.GeometricallyComparable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

/**
 * A {@link BoundingBox2D} can be used to define from a set of minimum and maximum coordinates an
 * axis-aligned bounding box in the XY-plane.
 */
public class BoundingBox2D implements BoundingBox2DBasics, EpsilonComparable<BoundingBox2D>, Settable<BoundingBox2D>, GeometricallyComparable<BoundingBox2D>
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
   public static BoundingBox2D union(BoundingBox2DReadOnly boundingBoxOne, BoundingBox2DReadOnly boundingBoxTwo)
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
   public BoundingBox2D(BoundingBox2DReadOnly other)
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
    * Redefines this bounding box to be the same as the given {@code other}.
    *
    * @param other the bounding box used to redefine this bounding box. Not modified.
    */
   @Override
   public void set(BoundingBox2D other)
   {
      BoundingBox2DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public Point2D getMinPoint()
   {
      return minPoint;
   }

   /** {@inheritDoc} */
   @Override
   public Point2D getMaxPoint()
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
      return BoundingBox2DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(BoundingBox2DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((BoundingBox2DReadOnly) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
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
      return BoundingBox2DBasics.super.geometricallyEquals(other, epsilon);
   }
}
