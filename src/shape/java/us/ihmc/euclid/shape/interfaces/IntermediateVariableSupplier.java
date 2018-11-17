package us.ihmc.euclid.shape.interfaces;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

public interface IntermediateVariableSupplier
{
   Point3DBasics getPoint3D(int index);

   public static IntermediateVariableSupplier defaultIntermediateVariableSupplier()
   {
      return new IntermediateVariableSupplier()
      {
         @Override
         public Point3DBasics getPoint3D(int index)
         {
            return new Point3D();
         }
      };
   }

   public static IntermediateVariableSupplier garbageFreeIntermediateVariableSupplier()
   {
      return new IntermediateVariableSupplier()
      {
         private final List<Point3D> variablePool = new ArrayList<>();

         @Override
         public Point3DBasics getPoint3D(int index)
         {
            while (variablePool.size() <= index)
               variablePool.add(new Point3D());
            return variablePool.get(index);
         }
      };
   }
}
