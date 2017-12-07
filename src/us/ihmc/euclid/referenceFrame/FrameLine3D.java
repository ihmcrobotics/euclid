package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.interfaces.Line3DBasics;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class FrameLine3D extends FrameGeometryObject<FrameLine3D, Line3D> implements FrameLine3DReadOnly, Line3DBasics
{
   /** The line. */
   private final Line3D line;

   public FrameLine3D(Line3DReadOnly line3D)
   {
      this(ReferenceFrame.getWorldFrame(), line3D);
   }

   public FrameLine3D(ReferenceFrame referenceFrame, Line3DReadOnly line3D)
   {
      super(referenceFrame, new Line3D(line3D));
      line = getGeometryObject();
   }

   @Override
   public FramePoint3DReadOnly getPoint()
   {
      return new FramePoint3D(getReferenceFrame(), line.getPoint());
   }

   @Override
   public FrameVector3DReadOnly getDirection()
   {
      return new FrameVector3D(getReferenceFrame(), line.getDirection());
   }

   @Override
   public boolean hasPointBeenSet()
   {
      return line.hasPointBeenSet();
   }

   @Override
   public boolean hasDirectionBeenSet()
   {
      return line.hasDirectionBeenSet();
   }

   @Override
   public void setPoint(double pointOnLineX, double pointOnLineY, double pointOnLineZ)
   {
      line.setPoint(pointOnLineX, pointOnLineY, pointOnLineZ);
   }

   @Override
   public void setDirection(double lineDirectionX, double lineDirectionY, double lineDirectionZ)
   {
      line.setDirection(lineDirectionX, lineDirectionY, lineDirectionZ);
   }

   @Override
   public void setDirectionUnsafe(double lineDirectionX, double lineDirectionY, double lineDirectionZ)
   {
      line.setDirectionUnsafe(lineDirectionX, lineDirectionY, lineDirectionZ);
   }

   /**
    * Changes the point through which this line has to go.
    *
    * @param pointOnLine new point on this line. Not modified.
    */
   public void setPoint(FramePoint3DReadOnly pointOnLine)
   {
      checkReferenceFrameMatch(pointOnLine);
      Line3DBasics.super.setPoint(pointOnLine);
   }

   /**
    * Changes the direction of this line by setting it to the normalized value of the given vector.
    *
    * @param lineDirection new direction of this line. Not modified.
    * @throws RuntimeException if the new direction is unreasonably small.
    */
   public void setDirection(FrameVector3DReadOnly lineDirection)
   {
      checkReferenceFrameMatch(lineDirection);
      Line3DBasics.super.setDirection(lineDirection);
   }
}
