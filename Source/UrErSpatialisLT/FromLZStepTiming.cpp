#include "FromLZStepTiming.h"

#include "Dom/JsonObject.h"
#include "HAL/CriticalSection.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformTime.h"
#include "Misc/DateTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

namespace FromLZStepTiming
{
namespace
{
	struct FStepTimingRecord
	{
		int32 Sequence = 0;
		FString StepName;
		FString Category;
		FString Notes;
		FString ThreadName;
		double StartSeconds = 0.0;
		double EndSeconds = 0.0;
		double DurationMilliseconds = 0.0;
	};

	struct FPressTimingState
	{
		FString PressName;
		FString PressDir;
		FString StartedAt;
		double OriginSeconds = 0.0;
		double LastEndSeconds = 0.0;
		int32 NextSequence = 0;
		TArray<FStepTimingRecord> Records;
	};

	FCriticalSection GTimingLock;
	TMap<FString, FPressTimingState> GTimingByPressDir;

	FString TimingKey(const FString& PressDir)
	{
		return FPaths::ConvertRelativePathToFull(PressDir);
	}

	FString CurrentThreadName()
	{
		return IsInGameThread() ? TEXT("GameThread") : TEXT("WorkerThread");
	}

	void SaveTimingStateLocked(const FPressTimingState& State)
	{
		if (State.PressDir.IsEmpty())
		{
			return;
		}

		TSharedRef<FJsonObject> Root = MakeShared<FJsonObject>();
		Root->SetNumberField(TEXT("schema_version"), 1);
		Root->SetStringField(TEXT("press"), State.PressName);
		Root->SetStringField(TEXT("press_dir"), State.PressDir);
		Root->SetStringField(TEXT("generated_at"), FDateTime::Now().ToIso8601());
		Root->SetStringField(TEXT("started_at"), State.StartedAt);
		Root->SetNumberField(TEXT("record_count"), State.Records.Num());
		Root->SetNumberField(
			TEXT("total_recorded_wall_ms"),
			FMath::Max(0.0, (State.LastEndSeconds - State.OriginSeconds) * 1000.0));

		TArray<TSharedPtr<FJsonValue>> StepValues;
		StepValues.Reserve(State.Records.Num());
		for (const FStepTimingRecord& Record : State.Records)
		{
			TSharedRef<FJsonObject> StepObject = MakeShared<FJsonObject>();
			StepObject->SetNumberField(TEXT("sequence"), Record.Sequence);
			StepObject->SetStringField(TEXT("step"), Record.StepName);
			StepObject->SetStringField(TEXT("category"), Record.Category);
			StepObject->SetStringField(TEXT("thread"), Record.ThreadName);
			StepObject->SetNumberField(
				TEXT("start_ms_since_press"),
				FMath::Max(0.0, (Record.StartSeconds - State.OriginSeconds) * 1000.0));
			StepObject->SetNumberField(
				TEXT("end_ms_since_press"),
				FMath::Max(0.0, (Record.EndSeconds - State.OriginSeconds) * 1000.0));
			StepObject->SetNumberField(TEXT("duration_ms"), Record.DurationMilliseconds);
			if (!Record.Notes.IsEmpty())
			{
				StepObject->SetStringField(TEXT("notes"), Record.Notes);
			}
			StepValues.Add(MakeShared<FJsonValueObject>(StepObject));
		}
		Root->SetArrayField(TEXT("steps"), StepValues);

		const FString Path = TimingJsonPath(State.PressDir);
		IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), true);
		FString Json;
		TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Json);
		if (FJsonSerializer::Serialize(Root, Writer))
		{
			FFileHelper::SaveStringToFile(Json, *Path);
		}
	}
}

double NowSeconds()
{
	return FPlatformTime::Seconds();
}

FString TimingJsonPath(const FString& PressDir)
{
	return PressDir / TEXT("12_step_timings.json");
}

void ResetPress(const FString& PressDir, const FString& PressName)
{
	FScopeLock Lock(&GTimingLock);
	FPressTimingState State;
	State.PressName = PressName;
	State.PressDir = PressDir;
	State.StartedAt = FDateTime::Now().ToIso8601();
	State.OriginSeconds = NowSeconds();
	State.LastEndSeconds = State.OriginSeconds;
	GTimingByPressDir.Add(TimingKey(PressDir), State);
	SaveTimingStateLocked(State);
}

void RecordStep(
	const FString& PressDir,
	const FString& StepName,
	double StartSeconds,
	double EndSeconds,
	const FString& Category,
	const FString& Notes)
{
	if (PressDir.IsEmpty() || StepName.IsEmpty())
	{
		return;
	}

	if (EndSeconds < StartSeconds)
	{
		Swap(StartSeconds, EndSeconds);
	}

	FScopeLock Lock(&GTimingLock);
	const FString Key = TimingKey(PressDir);
	FPressTimingState& State = GTimingByPressDir.FindOrAdd(Key);
	if (State.PressDir.IsEmpty())
	{
		State.PressDir = PressDir;
		State.PressName = FPaths::GetCleanFilename(PressDir);
		State.StartedAt = FDateTime::Now().ToIso8601();
		State.OriginSeconds = StartSeconds;
		State.LastEndSeconds = StartSeconds;
	}

	FStepTimingRecord Record;
	Record.Sequence = State.NextSequence++;
	Record.StepName = StepName;
	Record.Category = Category;
	Record.Notes = Notes;
	Record.ThreadName = CurrentThreadName();
	Record.StartSeconds = StartSeconds;
	Record.EndSeconds = EndSeconds;
	Record.DurationMilliseconds = (EndSeconds - StartSeconds) * 1000.0;
	State.LastEndSeconds = FMath::Max(State.LastEndSeconds, EndSeconds);
	State.Records.Add(MoveTemp(Record));
	SaveTimingStateLocked(State);
}
}
