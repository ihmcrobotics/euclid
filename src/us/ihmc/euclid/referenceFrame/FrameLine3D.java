package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class FrameLine3D extends FrameGeometryObject<FrameLine3D, Line3D> implements FrameLine3DReadOnly
{
   /** The line. */
   private final Line3D line;

   public FrameLine3D(Line3D line3D)
   {
      this(ReferenceFrame.getWorldFrame(), line3D);
   }

   public FrameLine3D(ReferenceFrame referenceFrame, Line3D line3D)
   {
      super(referenceFrame, line3D);
      line = getGeometryObject();
   }

   @Override
   public Point3DReadOnly getPoint()
   {
      return line.getPoint();
   }

   @Override
   public Vector3DReadOnly getDirection()
   {
      return line.getDirection();
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
}
