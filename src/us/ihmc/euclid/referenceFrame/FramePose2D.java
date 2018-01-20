package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameOrientation2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

public class FramePose2D implements FramePose2DBasics, GeometryObject<FramePose2D>
{
   /** The reference frame is which this point is currently expressed. */
   private ReferenceFrame referenceFrame;
   /** Pose used to perform the operations. */
   private final Pose2D pose = new Pose2D();

   private final FixedFrameOrientation2DBasics orientationPart = new FixedFrameOrientation2DBasics()
   {
      @Override
      public void setYaw(double yaw)
      {
         pose.setYaw(yaw);
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return referenceFrame;
      }

      @Override
      public double getYaw()
      {
         return pose.getYaw();
      }

      @Override
      public void applyTransform(Transform transform)
      {
         pose.getOrientation().applyTransform(transform);
      }

      @Override
      public void applyInverseTransform(Transform transform)
      {
         pose.getOrientation().applyInverseTransform(transform);
      }
   };

   private final FixedFramePoint2DBasics positionPart = new FixedFramePoint2DBasics()
   {
      @Override
      public void setX(double x)
      {
         pose.setX(x);
      }

      @Override
      public void setY(double y)
      {
         pose.setY(y);
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return referenceFrame;
      }

      @Override
      public double getX()
      {
         return pose.getX();
      }

      @Override
      public double getY()
      {
         return pose.getY();
      }

      @Override
      public void applyTransform(Transform transform, boolean checkIfTransformInXYplane)
      {
         pose.getPosition().applyTransform(transform, checkIfTransformInXYplane);
      }

      @Override
      public void applyInverseTransform(Transform transform, boolean checkIfTransformInXYplane)
      {
         pose.getPosition().applyInverseTransform(transform, checkIfTransformInXYplane);
      }
   };

   public FramePose2D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public FramePose2D(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   public FramePose2D(FrameTuple2DReadOnly position, FrameOrientation2DReadOnly orientation)
   {
      setIncludingFrame(position, orientation);
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

   public FramePose2D(ReferenceFrame referenceFrame, FrameTuple2DReadOnly position, double yaw)
   {
      setIncludingFrame(referenceFrame, position, yaw);
   }
   
   public FramePose2D(ReferenceFrame referenceFrame, Tuple2DReadOnly position, double yaw)
   {
      setIncludingFrame(referenceFrame, position, yaw);
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
   public FixedFramePoint2DBasics getPosition()
   {
      return positionPart;
   }

   @Override
   public FixedFrameOrientation2DBasics getOrientation()
   {
      return orientationPart;
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
