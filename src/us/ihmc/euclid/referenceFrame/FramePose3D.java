package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;

public class FramePose3D implements FramePose3DBasics, GeometryObject<FramePose3D>
{
   private ReferenceFrame referenceFrame;
   /** Pose used to perform the operations. */
   private final Pose3D pose = new Pose3D();

   private final FixedFramePoint3DBasics positionPart = new FixedFramePoint3DBasics()
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
      public void setZ(double z)
      {
         pose.setZ(z);
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
      public double getZ()
      {
         return pose.getZ();
      }
   };

   private final FixedFrameQuaternionBasics orientationPart = new FixedFrameQuaternionBasics()
   {

      @Override
      public void setUnsafe(double qx, double qy, double qz, double qs)
      {
         pose.getOrientation().setUnsafe(qx, qy, qz, qs);
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return referenceFrame;
      }

      @Override
      public double getX()
      {
         return pose.getOrientation().getX();
      }

      @Override
      public double getY()
      {
         return pose.getOrientation().getY();
      }

      @Override
      public double getZ()
      {
         return pose.getOrientation().getZ();
      }

      @Override
      public double getS()
      {
         return pose.getOrientation().getS();
      }
   };

   public FramePose3D()
   {
      setToZero(ReferenceFrame.getWorldFrame());
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
   public FramePose3D(Pose3DReadOnly pose)
   {
      setIncludingFrame(ReferenceFrame.getWorldFrame(), pose);
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
   public FramePose3D(ReferenceFrame referenceFrame, Pose3DReadOnly pose)
   {
      setIncludingFrame(referenceFrame, pose);
   }

   public FramePose3D(FramePose3DReadOnly other)
   {
      setIncludingFrame(other);
   }

   @Override
   public void set(FramePose3D other)
   {
      FramePose3DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint3DBasics getPosition()
   {
      return positionPart;
   }

   /** {@inheritDoc} */
   @Override
   public FixedFrameQuaternionBasics getOrientation()
   {
      return orientationPart;
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
   public boolean epsilonEquals(FramePose3D other, double epsilon)
   {
      return FramePose3DBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean geometricallyEquals(FramePose3D other, double epsilon)
   {
      return FramePose3DBasics.super.geometricallyEquals(other, epsilon);
   }

   @Override
   public boolean equals(Object obj)
   {
      try
      {
         return super.equals((Pose3DReadOnly) obj);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   @Override
   public int hashCode()
   {
      return pose.hashCode();
   }
}
