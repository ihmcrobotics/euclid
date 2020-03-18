package us.ihmc.euclid.referenceFrame.api;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;

import us.ihmc.euclid.tools.EuclidCoreIOTools;

public class MethodSignature
{
   private String name;
   private int modifiers;
   private Class<?>[] parameterTypes;
   private Class<?> returnType;

   public MethodSignature()
   {
   }

   public MethodSignature(String name, Class<?>... parameterTypes)
   {
      setName(name);
      setParameterTypes(parameterTypes);
   }

   public MethodSignature(MethodSignature other)
   {
      set(other);
   }

   public MethodSignature(Method method)
   {
      set(method);
   }

   public void set(MethodSignature other)
   {
      name = other.getName();
      modifiers = other.getModifiers();
      parameterTypes = new Class<?>[other.getParameterCount()];
      for (int i = 0; i < other.getParameterCount(); i++)
         parameterTypes[i] = other.getParameterTypes()[i];
      returnType = other.getReturnType();
   }

   public void set(Method method)
   {
      name = method.getName();
      modifiers = method.getModifiers();
      parameterTypes = new Class<?>[method.getParameterCount()];
      for (int i = 0; i < method.getParameterCount(); i++)
         parameterTypes[i] = method.getParameterTypes()[i];
      returnType = method.getReturnType();
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public void setModifiers(int modifiers)
   {
      this.modifiers = modifiers;
   }

   public int getParameterCount()
   {
      return parameterTypes.length;
   }

   public void setParameterTypes(Class<?>[] parameterTypes)
   {
      this.parameterTypes = parameterTypes;
   }

   public void setParameterType(int index, Class<?> parameterType)
   {
      parameterTypes[index] = parameterType;
   }

   public void initializeParameterTypes(int numberOfParameters)
   {
      parameterTypes = new Class<?>[numberOfParameters];
   }

   public void setReturnType(Class<?> returnType)
   {
      this.returnType = returnType;
   }

   public String getName()
   {
      return name;
   }

   public int getModifiers()
   {
      return modifiers;
   }

   public boolean isStatic()
   {
      return Modifier.isStatic(modifiers);
   }

   public boolean isPublic()
   {
      return Modifier.isStatic(modifiers);
   }

   public Class<?>[] getParameterTypes()
   {
      return parameterTypes;
   }

   public Class<?> getParameterType(int index)
   {
      return parameterTypes[index];
   }

   public Class<?> getReturnType()
   {
      return returnType;
   }

   public String getMethodSimpleName()
   {
      return getMethodSimpleName(returnType, name, parameterTypes);
   }

   public static String getMethodSimpleName(Method method)
   {
      return getMethodSimpleName(method.getReturnType(), method.getName(), method.getParameterTypes());
   }

   public static String getMethodSimpleName(Class<?> returnType, String methodName, Class<?>... parameterTypes)
   {
      String returnTypeName = returnType == null ? "void" : returnType.getSimpleName();
      return returnTypeName + " " + methodName + EuclidCoreIOTools.getCollectionString("(", ")", ", ", Arrays.asList(parameterTypes), Class::getSimpleName);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof MethodSignature)
      {
         MethodSignature other = (MethodSignature) object;

         if (name == null ? other.name == null : !name.equals(other.name))
            return false;
         if (modifiers != other.modifiers)
            return false;
         if (!Arrays.equals(parameterTypes, other.parameterTypes))
            return false;
         if (returnType != other.returnType)
            return false;

         return true;
      }
      else
      {
         return false;
      }
   }
}
