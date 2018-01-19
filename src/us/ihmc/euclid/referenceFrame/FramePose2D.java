package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.interfaces.Orientation2DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;

public class FramePose2D implements FramePose2DBasics, GeometryObject<FramePose2D>
{
   /** The reference frame is which this point is currently expressed. */
   private ReferenceFrame referenceFrame;
   /** Pose used to perform the operations. */
   protected final Pose2D pose = new Pose2D();

   public FramePose2D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public FramePose2D(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   /**
    * Creates a new frame pose and initializes its current reference frame to
    * {@link ReferenceFrame#getWorldFrame()} and pose to {@code pose}.
    * <p>
    * The given {@code pose}'s reference is saved internally for performing all the future
    * operations with this {@code FramePose3D}.
    * </p>
    *
    * @param pose the pose that is to be used internally. Copied. Not modified.
    */
   public FramePose2D(FramePose2DReadOnly pose)
   {
      this(ReferenceFrame.getWorldFrame(), pose);
   }

   /**
    * Creates a new frame pose and initializes its current reference frame and pose.
    * <p>
    * The given {@code pose}'s reference is saved internally for performing all the future
    * operations with this {@code FramePose3D}.
    * </p>
    *
    * @param referenceFrame the initial reference frame in which the given pose is expressed in.
    * @param pose the pose that is to be used internally. Copied. Not modified.
    */
   public FramePose2D(ReferenceFrame referenceFrame, Pose2DReadOnly pose)
   {
      setIncludingFrame(referenceFrame, pose);
   }

   @Override
   public void set(FramePose2D other)
   {
      FramePose2DBasics.super.set(other);
   }

   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   @Override
   public Point2DBasics getPosition()
   {
      return pose.getPosition();
   }

   @Override
   public Orientation2DBasics getOrientation()
   {
      return pose.getOrientation();
   }

   @Override
   public boolean equals(Object obj)
   {
      try
      {
         return equals((FramePose2DReadOnly) obj);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   @Override
   public boolean epsilonEquals(FramePose2D other, double epsilon)
   {
      return FramePose2DBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean geometricallyEquals(FramePose2D other, double epsilon)
   {
      return FramePose2DBasics.super.geometricallyEquals(other, epsilon);
   }
}
