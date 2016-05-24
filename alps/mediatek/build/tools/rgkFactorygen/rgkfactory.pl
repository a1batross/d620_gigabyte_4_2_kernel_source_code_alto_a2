#!/usr/local/bin/perl -w
# Ragentek version 1.0   lidong.zhou@ragentek.com

use File::Basename;
#use strict;
my $LOCAL_PATH;
BEGIN
{
    $LOCAL_PATH = dirname($0);
}

use lib "$LOCAL_PATH/../Spreadsheet";
use lib "$LOCAL_PATH/../";
require 'ParseExcel.pm';
use pack_dep_gen;

#****************************************************************************
# Customization Field
#****************************************************************************
my $PLATFORM = $ENV{MTK_PLATFORM};# MTxxxx
my $PROJECT = $ENV{PROJECT};
my $FULL_PROJECT = $ENV{FULL_PROJECT};

my @FactoryList;
print "\$PLATFORM=$PLATFORM,\$PROJECT=$PROJECT,\$FULL_PROJECT=$FULL_PROJECT\n";
PrintDependModule($0);
#****************************************************************************
# Main Thread Field
#****************************************************************************
    &ReadFactoryExcelFile();
    &GenFactoryHeaderFile();
    print "Factorygen END\n";
    exit 0;


#****************************************************************************
# Subfunction Field
#****************************************************************************
sub ReadFactoryExcelFile
{  
    my @all_column=[];
    my $FACTORY_DEVICE_LIST_XLS = "mediatek/build/tools/rgkFactorygen/RgkFactoryList.xls";
    my $FULL_PROJECT_LIST_XLS = "mediatek/config/${FULL_PROJECT}/RgkFactoryList.xls";
    my $PROJECT_LIST_XLS = "mediatek/config/${PROJECT}/RgkFactoryList.xls";
    my $SheetName = "Factory";
    my $parser = Spreadsheet::ParseExcel->new();

    my $Book = $parser->Parse($FULL_PROJECT_LIST_XLS);
   
    if(!defined $Book)
    {
        print "get partition sheet from $FULL_PROJECT_LIST_XLS failed, try $PROJECT_LIST_XLS...\n";
	$Book = $parser->Parse($PROJECT_LIST_XLS);
	
    }

    if(!defined $Book)
    {
        print "get partition sheet from PROJECT_LIST_XLS failed, try $FACTORY_DEVICE_LIST_XLS...\n";
	$Book = $parser->Parse($FACTORY_DEVICE_LIST_XLS);
	
    }

    PrintDependency($FACTORY_DEVICE_LIST_XLS);
    my $sheet = $Book->Worksheet($SheetName);
    my %COLUMN_LIST;
    my $tmp;
    my $row;
    my $col;

	for($col = 0, $row = 0,$tmp = &xls_cell_value($sheet, $row, $col); $tmp; $col++, $tmp = &xls_cell_value($sheet, $row, $col))
	{
		$COLUMN_LIST{$tmp} = $col;
	}
		@all_column=sort (keys(%COLUMN_LIST));
	
	for($row = 1,$tmp = &xls_cell_value($sheet, $row, $COLUMN_LIST{ITEM});$tmp;$row++,$tmp = &xls_cell_value($sheet, $row, $COLUMN_LIST{ITEM}))
	{
		foreach $i (@all_column){
			$FactoryList[$row-1]{$i}=&xls_cell_value($sheet, $row, $COLUMN_LIST{$i});
		}

	}

}

#****************************************************************************
# Subfunction Field
#****************************************************************************

sub GenFactoryHeaderFile()
{

    my $Factory_LIST_DEFINE_H_NAME = "mediatek/custom/${FULL_PROJECT}/factory/inc/cust.h";
        open(FD, ">$Factory_LIST_DEFINE_H_NAME") or &error_handler("open $Factory_LIST_DEFINE_H_NAME fail\n", __FILE__, __LINE__);
        print FD "#ifndef FTM_CUST_H \n#define FTM_CUST_H\n\n";

	for($index=0;$index<@FactoryList;$index++){

		if ($FactoryList[$index]->{CHOOSE} eq "yes")
		{
			print  FD "$FactoryList[$index]->{MACROS}  \n";

	        }else
		{
                        print  FD "//$FactoryList[$index]->{MACROS}  \n";
		}
			

	};

    print FD &struct_include_head();
    print FD "#endif /* FTM_CUST_H */\n";
    close FD;

}

#****************************************************************************************
# subroutine:  xls_cell_value
# return:      Excel cell value no matter it's in merge area or not, and in windows or not
# input:       $Sheet:  Specified Excel Sheet
# input:       $row:    Specified row number
# input:       $col:    Specified column number
#****************************************************************************************
sub xls_cell_value()
{
    my($Sheet, $row, $col) = @_;
    my $cell = $Sheet->get_cell($row, $col);
    if (defined $cell)
    {
        return $cell->Value();
    } else
    {
        print "$Sheet: row=$row, col=$col undefined\n";
        return;
    }
}

#****************************************************************************
# subroutine:  error_handler
# input:       $error_msg:     error message
#****************************************************************************
sub error_handler()
{
    my($error_msg, $file, $line_no) = @_;
    my $final_error_msg = "FactorydGen ERROR: $error_msg at $file line $line_no\n";
    print $final_error_msg;
    die $final_error_msg;
}

#****************************************************************************
#add head and other
#
#****************************************************************************

sub struct_include_head()
{
    my $template = <<"__TEMPLATE";

#include "cust_font.h"		/* common part */
#include "cust_keys.h"		/* custom part */
#include "cust_lcd.h"		/* custom part */
#include "cust_led.h"		/* custom part */
#include "cust_touch.h"         /* custom part */

__TEMPLATE

   return $template;
}




