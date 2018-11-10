package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameYawPitchRollBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameYawPitchRollReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class FrameYawPitchRoll implements FrameYawPitchRollBasics, GeometryObject<FrameYawPitchRoll>
{
   /** The reference frame is which this yaw-pitch-roll is currently expressed. */
   private ReferenceFrame referenceFrame;
   /** The yaw angle representing the first rotation around the z-axis. */
   private double yaw;
   /** The pitch angle representing the second rotation around the y-axis. */
   private double pitch;
   /** The roll angle representing the third rotation around the x-axis. */
   private double roll;

   /**
    * Creates a new frame yaw-pitch-roll, initializes its angles to zero and its reference frame to
    * {@link ReferenceFrame#getWorldFrame()}.
    */
   public FrameYawPitchRoll()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new frame yaw-pitch-roll, initializes its angles to zero, and its reference frame to
    * the {@code referenceFrame}.
    *
    * @param referenceFrame the initial frame for this frame yaw-pitch-roll.
    */
   public FrameYawPitchRoll(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   /**
    * Creates a new frame yaw-pitch-roll and initializes its angles {@code yaw}, {@code pitch}, and
    * {@code roll} in order from the given array and initializes its reference frame.
    *
    * @param referenceFrame the initial frame for this frame yaw-pitch-roll.
    * @param yawPitchRollArray the array containing this yaw-pitch-roll's angles. Not modified.
    */
   public FrameYawPitchRoll(ReferenceFrame referenceFrame, double[] yawPitchRollArray)
   {
      setIncludingFrame(referenceFrame, yawPitchRollArray);
   }

   /**
    * Creates a new frame yaw-pitch-roll and initializes such that it represents the same orientation
    * as the given {@code orientation3DReadOnly} and initializes its reference frame.
    *
    * @param referenceFrame the initial frame for this frame yaw-pitch-roll.
    * @param orientation3DReadOnly the orientation to initialize this yaw-pitch-roll. Not modified.
    */
   public FrameYawPitchRoll(ReferenceFrame referenceFrame, Orientation3DReadOnly orientation3DReadOnly)
   {
      setIncludingFrame(referenceFrame, orientation3DReadOnly);
   }

   /**
    * Creates a new frame yaw-pitch-roll and initializes such that it represents the same orientation
    * as the given {@code rotationVector} and initializes its reference frame.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param referenceFrame the initial frame for this frame yaw-pitch-roll.
    * @param rotationVector the rotation vector to initialize this yaw-pitch-roll. Not modified.
    */
   public FrameYawPitchRoll(ReferenceFrame referenceFrame, Vector3DReadOnly rotationVector)
   {
      setRotationVectorIncludingFrame(referenceFrame, rotationVector);
   }

   /**
    * Creates a new frame yaw-pitch-roll and initializes its angles to the given ones.
    *
    * @param referenceFrame the initial frame for this frame yaw-pitch-roll.
    * @param yaw the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll the angle to rotate about the x-axis.
    */
   public FrameYawPitchRoll(ReferenceFrame referenceFrame, double yaw, double pitch, double roll)
   {
      setYawPitchRollIncludingFrame(referenceFrame, yaw, pitch, roll);
   }

   /**
    * Creates a new frame yaw-pitch-roll and initializes it to {@code other}.
    *
    * @param other the frame yaw-pitch-roll to copy the components and reference frame from. Not
    *           modified.
    */
   public FrameYawPitchRoll(FrameYawPitchRollReadOnly other)
   {
      setIncludingFrame(other);
   }

   @Override
   public void set(FrameYawPitchRoll other)
   {
      FrameYawPitchRollBasics.super.set((FrameYawPitchRollReadOnly) other);
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public void setYaw(double yaw)
   {
      this.yaw = yaw;
   }

   /** {@inheritDoc} */
   @Override
   public void setPitch(double pitch)
   {
      this.pitch = pitch;
   }

   /** {@inheritDoc} */
   @Override
   public void setRoll(double roll)
   {
      this.roll = roll;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public double getYaw()
   {
      return yaw;
   }

   /** {@inheritDoc} */
   @Override
   public double getPitch()
   {
      return pitch;
   }

   /** {@inheritDoc} */
   @Override
   public double getRoll()
   {
      return roll;
   }

   /**
    * Tests on a per component basis, if this yaw-pitch-roll is exactly equal to {@code other}. A
    * failing test does not necessarily mean that the two yaw-pitch-rolls represent two different
    * orientations.
    * <p>
    * If the two yaw-pitch-rolls have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other yaw-pitch-roll to compare against this. Not modified.
    * @return {@code true} if the two yaw-pitch-rolls are exactly equal component-wise and are
    *         expressed in the same reference frame, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object other)
   {
      if (other instanceof FrameYawPitchRollReadOnly)
         return equals((FrameYawPitchRollReadOnly) other);
      else
         return false;
   }

   /**
    * Tests on a per component basis, if this yaw-pitch-roll is equal to {@code other} to an
    * {@code epsilon}. A failing test does not necessarily mean that the two yaw-pitch-rolls represent
    * two different orientations.
    * <p>
    * If the two yaw-pitch-rolls have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other tuple to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two yaw-pitch-rolls are equal and are expressed in the same reference
    *         frame, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(FrameYawPitchRoll other, double epsilon)
   {
      return FrameYawPitchRollBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same orientation to an {@code epsilon}.
    * <p>
    * Two yaw-pitch-roll are considered geometrically equal if the magnitude of their difference is
    * less than or equal to {@code epsilon}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other the other yaw-pitch-roll to compare against this. Not modified.
    * @param epsilon the maximum angle for the two quaternions to be considered equal.
    * @return {@code true} if the two yaw-pitch-roll represent the same geometry, {@code false}
    *         otherwise.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   @Override
   public boolean geometricallyEquals(FrameYawPitchRoll other, double epsilon)
   {
      return FrameYawPitchRollBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Provides a {@code String} representation of this frame quaternion as follows: (x, y, z,
    * s)-worldFrame.
    *
    * @return the {@code String} representing this frame quaternion.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getYawPitchRollString(this) + "-" + referenceFrame;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this frame
    * quaternion.
    *
    * @return the hash code value for this frame quaternion.
    */
   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = 31L * bits + Double.doubleToLongBits(yaw);
      bits = 31L * bits + Double.doubleToLongBits(pitch);
      bits = 31L * bits + Double.doubleToLongBits(roll);
      return (int) (bits ^ bits >> 32);
   }
}
