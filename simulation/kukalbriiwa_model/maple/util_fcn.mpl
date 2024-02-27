
# http://compgroups.net/comp.soft-sys.math.maple/code-generation-from-a-matrix/2418212
file_export:=proc(sym::~Matrix,name::string,folder_name::string,parameterlist::list)
     local s, m, tmp, f, i, rows, columns, param,abs_dir_path;
     s:=convert(indets(sym, 'symbol'),list):
     #Remove all elemens from s, which are part of the parameterlist
     for i from 1 to numelems(parameterlist) do
       s:=subs(parameterlist[i]=NULL,s);
     end do:

     #Important step to export. Larger expressions become troublesome
     #when being exported as "Matrix". An "array", on the other hand,
     #works fine. Therefore, the expression is converted to an array.
     #See
     #  http://compgroups.net/comp.soft-sys.math.maple/code-generation-from-a-matrix/2418212
     #for the original suggestion.
     #
     #Parameter "sym" is always converted to a Matrix by calling the
     #proc- also Vectors arrive as a Matrix. Interestingly,
     #codegen[makeproc]() works only if the row dimension is less or
     #equal to the column dimension. In that case, transpose.
     rows,columns:=LinearAlgebra[Dimension](sym);
     if rows>columns then
       m:=convert(LinearAlgebra[Transpose](sym),array);
     else
       m:=convert(sym,array):
     end;
     
     #Append the parameter struct "param" to the parameterlist only if
     #there are any constant parameters contained.
     if numelems(s)>0 then
       tmp:=codegen[makeproc](m,parameters=[op(parameterlist),param]):
     else
       tmp:=codegen[makeproc](m,parameters=parameterlist):
     end;
     tmp:=codegen[optimize](tmp):
     tmp:=CodeGeneration[Matlab](tmp,output=string,optimize=false):

     #Prepend all used parameters by "param." to make them fields of a struct.
     #For example, s__1x is replaced by param.s__1x
     #Elements, which are present in the parameterlist, have already been
     #removed from s
     for i from 1 to numelems(s) do
       tmp:=StringTools[RegSubs](cat("([ \\(\\-])(",convert(s[i],string),")([; \\(\\)])")="\\1param.\\2\\3",tmp);
     end do:

     #Replace the function name
     tmp:=StringTools[SubstituteAll](tmp,"function tmpreturn = tmp(",cat("function tmpreturn = ",name,"("));

     #If the matrix was transposed before, replace the last line (return value)
     #to transpose again.
     if rows>columns then
       tmp:=StringTools[SubstituteAll](tmp,"tmpreturn = m;","tmpreturn = m';");
     end;
     abs_dir_path :=cat(currentdir(),"\/..\/..\/matlab\/",folder_name,"\/"); 
     if FileTools[Exists](abs_dir_path) = false then
       mkdir(abs_dir_path)
     end;
     #Write the resulting string to the file "<name>.m"
     f:=FileTools[Text][Open](cat(abs_dir_path,name,".m"),create,overwrite):
     FileTools[Text][WriteString](f,tmp):
     FileTools[Text][Close](f):
   end proc:
file_export_C:=proc(sym::~Matrix,name::string,folder_name::string,parameterlist::list, type::string := ".hpp" )
     local s, m, tmp, f, i, rows, columns, param,abs_dir_path;
     s:=convert(indets(sym, 'symbol'),list):
     #Remove all elemens from s, which are part of the parameterlist
     for i from 1 to numelems(parameterlist) do
       s:=subs(parameterlist[i]=NULL,s);
     end do:

     #Important step to export. Larger expressions become troublesome
     #when being exported as "Matrix". An "array", on the other hand,
     #works fine. Therefore, the expression is converted to an array.
     #See
     #  http://compgroups.net/comp.soft-sys.math.maple/code-generation-from-a-matrix/2418212
     #for the original suggestion.
     #
     #Parameter "sym" is always converted to a Matrix by calling the
     #proc- also Vectors arrive as a Matrix. Interestingly,
     #codegen[makeproc]() works only if the row dimension is less or
     #equal to the column dimension. In that case, transpose.
     rows,columns:=LinearAlgebra[Dimension](sym);
     if rows>columns then
       m:=convert(LinearAlgebra[Transpose](sym),array);
     else
       m:=convert(sym,array):
     end;
     
     #Append the parameter struct "param" to the parameterlist only if
     #there are any constant parameters contained.
     if numelems(s)>0 then
       tmp:=codegen[makeproc](m,parameters=[op(parameterlist),param]):
     else
       tmp:=codegen[makeproc](m,parameters=parameterlist):
     end;
     tmp:=codegen[optimize](tmp):
     tmp:=CodeGeneration[C](tmp,output=string,optimize=false,deducetypes=false,defaulttype=float):

     #Prepend all used parameters by "param." to make them fields of a struct.
     #For example, s__1x is replaced by param.s__1x
     #Elements, which are present in the parameterlist, have already been
     #removed from s
     for i from 1 to numelems(s) do
       tmp:=StringTools[RegSubs](cat("([ \\(\\-])(",convert(s[i],string),")([; \\(\\)])")="\\1param.\\2\\3",tmp);
     end do:

     #Replace the function name
     tmp:=StringTools[SubstituteAll](tmp, " double param,", "const RobotParams& param,");
     if type = ".hpp" then
       tmp:=StringTools[SubstituteAll](tmp,"#include <math.h>","");
       tmp:=StringTools[SubstituteAll](tmp,"void tmp (",cat("template<typename Scalar> void ",name," ("));
       tmp:=StringTools[SubstituteAll](tmp, "double ", "Scalar ");
     end;
     #If the matrix was transposed before, replace the last line (return value)
     #to transpose again.
     if rows>columns then
       tmp:=StringTools[SubstituteAll](tmp,"tmpreturn = m;","tmpreturn = m';");
     end;
     
     #Write the resulting string to the file "<name>.c"r
     abs_dir_path :=cat(currentdir(),"\/..\/..\/cpp\/maple_generated\/",folder_name,"\/"); 
     if FileTools[Exists](abs_dir_path) = false then
       mkdir(abs_dir_path)
     end;
     f:=FileTools[Text][Open](cat(abs_dir_path, name,type),create,overwrite):
     FileTools[Text][WriteString](f,tmp):
     FileTools[Text][Close](f):
   end proc:
save file_export_C,file_export, "util_fcn.mpl":


