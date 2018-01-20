package us.ihmc.euclid.geometry;

import us.ihmc.euclid.geometry.interfaces.Orientation2DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose2DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

/**
 * A {@code Pose2D} represents a position and orientation in the XY-plane.
 */
public class Pose2D implements Pose2DBasics, GeometryObject<Pose2D>
{
   /** The position part of this pose 2D. */
   private final Point2D position = new Point2D();
   /** The orientation part of this pose 2D. */
   private final Orientation2D orientation = new Orientation2D();

   /**
    * Creates a new pose 2D initialized with its position at (0, 0) and orientation at 0.
    */
   public Pose2D()
   {
   }

   /**
    * Creates a new pose 2D and initializes it with the given parameters.
    *
    * @param x the x-coordinate of the position.
    * @param y the y-coordinate of the position.
    * @param yaw the angle in radians for the orientation.
    */
   public Pose2D(double x, double y, double yaw)
   {
      set(x, y, yaw);
   }

   /**
    * Creates a new pose 2D and initializes it to {@code other}.
    *
    * @param other the other pose 2D used to initialize this. Not modified.
    */
   public Pose2D(Pose2DReadOnly other)
   {
      set(other);
   }

   public Pose2D(Tuple2DReadOnly position, double yaw)
   {
      set(position, yaw);
   }

   /**
    * Creates a new pose 2D and initializes it with the given parameters.
    *
    * @param position tuple used to initialize the position part of this pose. Not modified.
    * @param orientation used to initialize the orientation part of this pose. Not modified.
    */
   public Pose2D(Tuple2DReadOnly position, Orientation2D orientation)
   {
      set(position, orientation);
   }

   /** {@inheritDoc} */
   @Override
   public Point2DBasics getPosition()
   {
      return position;
   }

   /** {@inheritDoc} */
   @Override
   public Orientation2DBasics getOrientation()
   {
      return orientation;
   }

   /**
    * Sets this pose 2D to the {@code other} pose 2D.
    *
    * @param other the other pose 2D. Not modified.
    */
   @Override
   public void set(Pose2D other)
   {
      Pose2DBasics.super.set(other);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method
    * returns {@link #equals(Pose2DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object obj)
   {
      try
      {
         return equals((Pose2DReadOnly) obj);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Tests on a per-component basis if this pose is equal to {@code other} with the tolerance
    * {@code epsilon}.
    *
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two poses are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Pose2D other, double epsilon)
   {
      return Pose2DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two poses are geometrically
    * similar.
    * <p>
    * Two poses are geometrically equal if both their position and orientation are geometrically
    * equal.
    * </p>
    *
    * @param other the pose to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two poses represent the same geometry, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(Pose2D other, double epsilon)
   {
      return Pose2DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Provides a {@code String} representation of this pose 2D as follows:<br>
    * Pose 2D: position = (x, y), orientation = (yaw)
    *
    * @return the {@code String} representing this pose 2D.
    */
   @Override
   public String toString()
   {
      return EuclidGeometryIOTools.getPose2DString(this);
   }
}
