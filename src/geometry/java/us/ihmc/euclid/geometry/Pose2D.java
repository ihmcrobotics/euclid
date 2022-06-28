package us.ihmc.euclid.geometry;

import us.ihmc.euclid.geometry.interfaces.Pose2DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.Orientation2D;
import us.ihmc.euclid.orientation.interfaces.Orientation2DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

/**
 * A {@code Pose2D} represents a position and orientation in the XY-plane.
 */
public class Pose2D implements Pose2DBasics
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
    * @param x   the x-coordinate of the position.
    * @param y   the y-coordinate of the position.
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

   /**
    * Creates a new pose 2D and initializes it from {@code pose3DReadOnly}
    *
    * @param pose3DReadOnly the pose 3D used to initialize this pose 2D. Not modified.
    */
   public Pose2D(Pose3DReadOnly pose3DReadOnly)
   {
      set(pose3DReadOnly);
   }

   /**
    * Creates a new pose 2D and initializes it from the given {@code position} and {@code yaw} angle.
    *
    * @param position the tuple used to initialize this pose's position. Not modified.
    * @param yaw      the angle used to initialize the pose's orientation.
    */
   public Pose2D(Tuple2DReadOnly position, double yaw)
   {
      set(position, yaw);
   }

   /**
    * Creates a new pose 2D and initializes it with the given parameters.
    *
    * @param position    tuple used to initialize the position part of this pose. Not modified.
    * @param orientation used to initialize the orientation part of this pose. Not modified.
    */
   public Pose2D(Tuple2DReadOnly position, Orientation2DReadOnly orientation)
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
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(Pose2DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Pose2DReadOnly)
         return equals((Pose2DReadOnly) object);
      else
         return false;
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
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this pose 2D.
    *
    * @return the hash code value for this pose 2D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(position, orientation);
   }
}
